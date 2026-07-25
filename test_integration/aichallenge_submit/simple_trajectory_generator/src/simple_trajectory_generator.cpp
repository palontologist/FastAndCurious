// Copyright 2023 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

class CSVToTrajectory : public rclcpp::Node
{
public:
  CSVToTrajectory() : Node("csv_to_trajectory_node")
  {
    const auto rb_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
    pub_ = this->create_publisher<Trajectory>("trajectory", rb_qos);
    set_parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&CSVToTrajectory::on_parameter_event, this, std::placeholders::_1));


declare_parameter("csv_path", "");
    z_= declare_parameter<float>("z");
    lateral_offset_ = declare_parameter<float>("lateral_offset", 0.0);
    std::string csv_path = get_parameter("csv_path").as_string();
    
    if (csv_path.empty()) {
      RCLCPP_ERROR(get_logger(), "CSV path is not specified");
      return;
    }
    
    if (!loadCSVTrajectory(csv_path)) {
      RCLCPP_ERROR(get_logger(), "Failed to load CSV file: %s", csv_path.c_str());
      return;
    }
    
    RCLCPP_INFO(get_logger(), "Loaded trajectory from CSV with %zu points, lateral_offset=%.2f", csv_trajectory_.points.size(), lateral_offset_);

    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::seconds(1),
      std::bind(&CSVToTrajectory::publish_trajectory, this));

  }

private:
  bool loadCSVTrajectory(const std::string & csv_path)
  {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
      return false;
    }
    
    std::string line;
    std::getline(file, line);
    
    csv_trajectory_.header.stamp = this->now();
    csv_trajectory_.header.frame_id = "map";

    csv_trajectory_.points.clear();
    
    // Load all raw points first
    struct RawPoint {
      double x, y, qx, qy, qz, qw, vel;
    };
    std::vector<RawPoint> raw_points;
    
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string token;
      std::vector<double> values;
      
      while (std::getline(ss, token, ',')) {
        values.push_back(std::stod(token));
      }
      
      if (values.size() != 8) {
        RCLCPP_WARN(get_logger(), "Invalid CSV line format, expected 8 values");
        continue;
      }
      
      raw_points.push_back({values[0], values[1], values[3], values[4], values[5], values[6], values[7]});
    }
    
    if (raw_points.empty()) {
      return false;
    }
    
    // Apply lateral offset
    const double offset = lateral_offset_;
    
    for (size_t i = 0; i < raw_points.size(); ++i) {
      const auto & rp = raw_points[i];
      
      // Calculate heading from quaternion
      double yaw = std::atan2(2.0 * (rp.qw * rp.qz + rp.qx * rp.qy), 
                               1.0 - 2.0 * (rp.qy * rp.qy + rp.qz * rp.qz));
      
      // Apply lateral offset perpendicular to heading
      double offset_x = -offset * std::sin(yaw);
      double offset_y = offset * std::cos(yaw);
      
      TrajectoryPoint point;
      point.pose.position.x = rp.x + offset_x;
      point.pose.position.y = rp.y + offset_y;
      point.pose.position.z = z_;

      point.pose.orientation.x = rp.qx;
      point.pose.orientation.y = rp.qy;
      point.pose.orientation.z = rp.qz;
      point.pose.orientation.w = rp.qw;
      
      point.longitudinal_velocity_mps = rp.vel;
      point.lateral_velocity_mps = 0.0;
      point.acceleration_mps2 = 0.0;
      point.heading_rate_rps = 0.0;
      
      csv_trajectory_.points.push_back(point);
    }
    
    return !csv_trajectory_.points.empty();
  }
  
  void publish_trajectory()
  {
    if (csv_trajectory_.points.empty()) {
      RCLCPP_WARN(get_logger(), "No trajectory points to publish");
      return;
    }
    
    csv_trajectory_.header.stamp = this->now();
    pub_->publish(csv_trajectory_);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(), 60000 /*ms*/, "Published trajectory with %zu points", csv_trajectory_.points.size());
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_event(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    for (const auto & param : parameters) {
      if (param.get_name() == "csv_path") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          std::string new_csv_path = param.as_string();
          // new_csv_pathがFileSystemのパスであることを確認
          if (!std::filesystem::exists(new_csv_path)) {
            RCLCPP_ERROR(get_logger(), "File does not exist: '%s'", new_csv_path.c_str());
            result.successful = false;
            result.reason = "File does not exist.";
            continue;
          }

          if (new_csv_path != current_csv_path_) {
            RCLCPP_INFO(get_logger(), "csv_path parameter changed from '%s' to '%s'", 
                        current_csv_path_.c_str(), new_csv_path.c_str());
            
            // 新しいCSVファイルの読み込みを試みる
            if (loadCSVTrajectory(new_csv_path)) {
              current_csv_path_ = new_csv_path;
              RCLCPP_INFO(get_logger(), "Successfully loaded new trajectory from CSV: %s with %zu points", 
                          current_csv_path_.c_str(), csv_trajectory_.points.size());
            } else {
              RCLCPP_ERROR(get_logger(), "Failed to load new CSV file: %s. Keeping old trajectory.", new_csv_path.c_str());
              result.successful = false;
              result.reason = "Failed to load new CSV file.";
            }
          }
        } else {
          RCLCPP_WARN(get_logger(), "Parameter 'csv_path' received with wrong type. Expected string.");
          result.successful = false;
          result.reason = "Invalid type for csv_path parameter.";
        }
      } else if (param.get_name() == "z") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE || param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          z_ = static_cast<float>(param.as_double());
          RCLCPP_INFO(get_logger(), "z parameter changed to %f", z_);
        } else {
          RCLCPP_WARN(get_logger(), "Parameter 'z' received with wrong type. Expected float/double.");
          result.successful = false;
          result.reason = "Invalid type for z parameter.";
        }
      } else if (param.get_name() == "lateral_offset") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE || param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          lateral_offset_ = static_cast<float>(param.as_double());
          RCLCPP_INFO(get_logger(), "lateral_offset parameter changed to %f", lateral_offset_);
          // Reload trajectory with new offset
          if (!current_csv_path_.empty()) {
            loadCSVTrajectory(current_csv_path_);
          }
        } else {
          RCLCPP_WARN(get_logger(), "Parameter 'lateral_offset' received with wrong type. Expected float/double.");
          result.successful = false;
          result.reason = "Invalid type for lateral_offset parameter.";
        }
      }
    }
    return result;
  }
  
  rclcpp::Publisher<Trajectory>::SharedPtr pub_;
rclcpp::TimerBase::SharedPtr timer_;
  Trajectory csv_trajectory_;
  float z_;
  float lateral_offset_;
  std::string current_csv_path_;
  OnSetParametersCallbackHandle::SharedPtr set_parameter_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CSVToTrajectory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
