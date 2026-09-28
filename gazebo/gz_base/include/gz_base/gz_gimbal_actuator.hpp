#ifndef GZ_BASE__GZ_GIMBAL_ACTUATOR_HPP_
#define GZ_BASE__GZ_GIMBAL_ACTUATOR_HPP_

#include <memory>
#include <string>

#include "ignition/transport/Node.hh"
#include "hardware_interface.hpp"
#include "robot_interfaces/msg/gimbal.hpp"

namespace gz_base
{

class IgnGimbalActuator : public Actuator<robot_interfaces::msg::Gimbal>
{
public:
  IgnGimbalActuator(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string & gz_pitch_topic,
    const std::string & gz_big_yaw_topic,
    const std::string & gz_small_yaw_topic,
    const std::string & gz_roll_topic);

  void set(const robot_interfaces::msg::Gimbal & data) override;
  void enable(bool enable) {enable_ = enable;}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  std::unique_ptr<ignition::transport::Node::Publisher> gz_pitch_pub_;
  std::unique_ptr<ignition::transport::Node::Publisher> gz_big_yaw_pub_;
  std::unique_ptr<ignition::transport::Node::Publisher> gz_small_yaw_pub_;
  std::unique_ptr<ignition::transport::Node::Publisher> gz_roll_pub_;
  bool enable_{false};
};

}  // namespace gz_base

#endif  // GZ_BASE__GZ_GIMBAL_ACTUATOR_HPP_
