#pragma once

#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "convert.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"

// namespace UT = UNITREE_LEGGED_SDK;
namespace aist_intern2023
{
class UnitreeUDPSender : public rclcpp::Node
{
private:
  UT::UDP high_udp_;
  UT::HighCmd send_cmd_;

  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr high_state_pub_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr high_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::TimerBase::SharedPtr main_timer_;

public:
  UnitreeUDPSender(const rclcpp::NodeOptions & options) : UnitreeUDPSender("", options) {}
  UnitreeUDPSender(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("unitree_udp_sender_node", name_space, options),
    high_udp_(8090, "192.168.12.1", 8082, sizeof(UT::HighCmd), sizeof(UT::HighState))
  {
    using namespace std::chrono_literals;
    UT::HighCmd init_cmd = {0};
    high_udp_.InitCmdData(init_cmd);

    high_state_pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighState>(
      "high_state", rclcpp::QoS(1));
    high_cmd_sub_ = this->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>(
      "high_cmd", rclcpp::QoS(1),
      std::bind(&UnitreeUDPSender::high_cmd_cb, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(1),
      std::bind(&UnitreeUDPSender::cmd_vel_cb, this, std::placeholders::_1));
    main_timer_ = this->create_wall_timer(0.2ms, [this]() {
      high_udp_.SetSend(send_cmd_);
      high_udp_.Send();
      ros2_unitree_legged_msgs::msg::HighState state_msg;
      UT::HighState state;
      high_udp_.Recv();
      high_udp_.GetRecv(state);
      state_msg = state2rosMsg(state);
      high_state_pub_->publish(state_msg);
    });
  }

  void high_cmd_cb(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
  {
    UT::HighCmd cmd = rosMsg2Cmd(msg);

    high_udp_.SetSend(cmd);
    high_udp_.Send();

    ros2_unitree_legged_msgs::msg::HighState state_msg;
    UT::HighState state;
    high_udp_.Recv();
    high_udp_.GetRecv(state);
    state_msg = state2rosMsg(state);

    high_state_pub_->publish(state_msg);
  }

  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) { send_cmd_ = rosMsg2Cmd(msg); }
};
}  // namespace aist_intern2023
