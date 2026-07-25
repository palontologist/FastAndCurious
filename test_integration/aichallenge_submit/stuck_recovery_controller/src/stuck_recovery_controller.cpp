#include "stuck_recovery_controller/stuck_recovery_controller.hpp"

#include <cmath>
#include <functional>
#include <memory>

namespace stuck_recovery_controller
{
namespace
{

constexpr float kStuckSpeedThreshold = 0.05;       // More sensitive
constexpr double kStuckDurationSec = 0.3;           // Faster detection
constexpr float kCommandSpeedThreshold = 0.5;       // Lower threshold
constexpr float kCommandAccelerationThreshold = 0.2;
constexpr float kMovingSpeedThreshold = 0.3;
constexpr double kReverseDurationSec = 1.2;         // Quicker reverse
constexpr double kReverseTurnDurationSec = 0.4;     // Turn while reversing
constexpr float kReverseSpeed = -4.0;               // Faster reverse
constexpr float kReverseAccel = 3.0;                // Harder reverse
constexpr float kReverseSteerAngle = 0.35;          // More counter-steer
constexpr float kBoostSpeed = 26.0;                 // Match race pace
constexpr float kBoostAccel = 6.0;                  // Violent boost
constexpr double kBoostDurationSec = 2.5;           // Longer boost
constexpr double kStartupGraceSec = 8.0;            // Shorter grace

}  // namespace

StuckRecoveryController::StuckRecoveryController() : Node("stuck_recovery_controller")
{
  control_pub_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", 1);
  gear_pub_ = create_publisher<GearCommand>("/control/command/gear_cmd", 1);
  node_start_time_ = this->now();

  nominal_sub_ = create_subscription<AckermannControlCommand>(
    "/control/command/nominal_control_cmd", 1,
    std::bind(&StuckRecoveryController::onNominalCommand, this, std::placeholders::_1));
  velocity_sub_ = create_subscription<VelocityReport>(
    "/vehicle/status/velocity_status", 1,
    [this](const VelocityReport::ConstSharedPtr msg) {
      latest_velocity_ = msg->longitudinal_velocity;
    });
}

void StuckRecoveryController::onNominalCommand(
  const AckermannControlCommand::ConstSharedPtr msg)
{
  const auto now = this->now();
  if (runRecovery(now)) {
    return;
  }
  control_pub_->publish(*msg);
  updateStuckDetection(*msg, now);
}

void StuckRecoveryController::updateStuckDetection(
  const AckermannControlCommand & command, const rclcpp::Time & now)
{
  // Startup grace period - ignore stuck detection for first kStartupGraceSec
  if ((now - node_start_time_).seconds() < kStartupGraceSec) {
    return;
  }

  const float velocity = latest_velocity_;
  // Require movement once to avoid detecting the initial stationary state as stuck.
  if (velocity >= kMovingSpeedThreshold) {
    moving_observed_ = true;
  }

  if (
    !moving_observed_ || command.longitudinal.speed < kCommandSpeedThreshold ||
    command.longitudinal.acceleration < kCommandAccelerationThreshold)
  {
    stuck_start_time_.reset();
    return;
  }

  if (std::abs(velocity) <= kStuckSpeedThreshold) {
    if (!stuck_start_time_.has_value()) {
      stuck_start_time_ = now;
    } else if ((now - stuck_start_time_.value()).seconds() >= kStuckDurationSec) {
      stuck_start_time_.reset();
      recovery_start_time_ = now;
      RCLCPP_INFO(get_logger(), "stuck detected: velocity=%.3f", velocity);
    }
  } else {
    stuck_start_time_.reset();
  }
}

bool StuckRecoveryController::runRecovery(const rclcpp::Time & now)
{
  if (!recovery_start_time_.has_value()) {
    return false;
  }

  const double elapsed = (now - recovery_start_time_.value()).seconds();

  // PHASE 1: Reverse with steering to pull away from wall (1.5s)
  if (elapsed < kReverseDurationSec) {
    publishGear(GearCommand::REVERSE);
    // First half: steer one way, second half: straighten out
    float steer = (elapsed < kReverseTurnDurationSec) ? kReverseSteerAngle : 0.0;
    publishCommand(kReverseSpeed, kReverseAccel, steer);
    return true;
  }

  // PHASE 2: Drive forward, boost to race speed with steering (2s)
  if (elapsed < kReverseDurationSec + kBoostDurationSec) {
    publishGear(GearCommand::DRIVE);
    // Slight steering to get back on track
    float steer = (elapsed < kReverseDurationSec + 0.5) ? -kReverseSteerAngle : 0.0;
    publishCommand(kBoostSpeed, kBoostAccel, steer);
    return true;
  }

  // PHASE 3: Recovery complete, resume nominal commands
  publishGear(GearCommand::DRIVE);
  recovery_start_time_.reset();
  return false;
}

void StuckRecoveryController::publishCommand(float speed, float acceleration, float steer_angle)
{
  const auto stamp = this->now();
  AckermannControlCommand msg;
  msg.stamp = stamp;
  msg.lateral.stamp = stamp;
  msg.lateral.steering_tire_angle = steer_angle;
  msg.lateral.steering_tire_rotation_rate = 2.0;
  msg.longitudinal.stamp = stamp;
  msg.longitudinal.speed = speed;
  msg.longitudinal.acceleration = acceleration;
  control_pub_->publish(msg);
}

void StuckRecoveryController::publishGear(std::uint8_t command)
{
  GearCommand msg;
  msg.stamp = this->now();
  msg.command = command;
  gear_pub_->publish(msg);
}

}  // namespace stuck_recovery_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stuck_recovery_controller::StuckRecoveryController>());
  rclcpp::shutdown();
  return 0;
}
