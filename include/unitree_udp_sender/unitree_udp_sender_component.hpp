#pragma once

#include <rclcpp/rclcpp.hpp>

#include "convert.h"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

namespace UT = UNITREE_LEGGED_SDK;
namespace aist_intern2023
{
class UnitreeUDPSender : public rclcpp::Node
{
private:
  UT::UDP high_udp_;
  UT::HighCmd high_cmd_ = {0};

  // rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr high_cmd_sub_;

public:
  UnitreeUDPSender(const rclcpp::NodeOptions & options) : UnitreeUDPSender("", options) {}
  UnitreeUDPSender(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("unitree_udp_sender_node", name_space, options),
    high_udp_(8090, "192.168.123.161", 8082, sizeof(UT::HighCmd), sizeof(UT::HighState))
  {
    high_udp_.InitCmdData(high_cmd_);
  }
};
}  // namespace aist_intern2023
