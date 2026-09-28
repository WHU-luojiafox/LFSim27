// Copyright 2021 RoboMaster-OSS
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

#ifndef GZ_BASE__GZ_GIMBAL_ENCODER_HPP_
#define GZ_BASE__GZ_GIMBAL_ENCODER_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <map>
#include <vector>

#include "ignition/transport/Node.hh"
#include "hardware_interface.hpp"
#include "robot_interfaces/msg/gimbal.hpp"

namespace gz_base
{

class IgnGimbalEncoder
{
public:
  IgnGimbalEncoder(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string & gz_joint_state_topic);
  ~IgnGimbalEncoder() {}

public:
  void enable(bool enable) {enable_ = enable;}
  Sensor<robot_interfaces::msg::Gimbal>::SharedPtr get_position_sensor() {return position_sensor_;}
  Sensor<robot_interfaces::msg::Gimbal>::SharedPtr get_velocity_sensor() {return velocity_sensor_;}

private:
  void gz_Joint_state_cb(const ignition::msgs::Model & msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  bool enable_{false};
  // info
  std::shared_ptr<DataSensor<robot_interfaces::msg::Gimbal>> position_sensor_;
  std::shared_ptr<DataSensor<robot_interfaces::msg::Gimbal>> velocity_sensor_;
};


}  // namespace gz_base

#endif  // GZ_BASE__GZ_GIMBAL_ENCODER_HPP_
