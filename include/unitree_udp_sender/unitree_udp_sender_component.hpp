#pragma once

#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "convert.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"

// namespace UT = UNITREE_LEGGED_SDK;

// emg state (bool)
// - True -> emg stop
// - False -> can move
namespace aist_intern2023
{
class UnitreeUDPSender : public rclcpp::Node
{
private:
  // UT::UDP high_udp_;
  std::shared_ptr<UT::UDP> high_udp_;
  UT::HighCmd send_cmd_;

  bool cr_emg_state_ = false;

  std::string target_ip_address;
  uint16_t local_port;
  uint16_t target_port;

  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr high_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cr_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cr_pos_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr high_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emg_switch_srv_;

  rclcpp::TimerBase::SharedPtr main_timer_;

  std_msgs::msg::Header make_header(rclcpp::Time ts, std::string frame)
  {
    std_msgs::msg::Header ret;
    ret.stamp = ts;
    ret.frame_id = frame;
    return ret;
  }

public:
  UnitreeUDPSender(const rclcpp::NodeOptions & options) : UnitreeUDPSender("", options) {}
  UnitreeUDPSender(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("unitree_udp_sender_node", name_space, options)  //,
    // high_udp_(8090, "192.168.123.161", 8082, sizeof(UT::HighCmd), sizeof(UT::HighState))
  {
    using namespace std::chrono_literals;

    declare_parameter("udp_settings.target_ip_address", "192.168.123.161");
    target_ip_address = get_parameter("udp_settings.target_ip_address").as_string();
    declare_parameter("udp_settings.local_port", 8090);
    local_port = get_parameter("udp_settings.local_port").as_int();
    declare_parameter("udp_settings.target_port", 8082);
    target_port = get_parameter("udp_settings.target_port").as_int();

    // high_udp_ = UT::UDP(
    //   local_port, target_ip_address.c_str(), target_port, sizeof(UT::HighCmd),
    //   sizeof(UT::HighState));
    high_udp_ = std::make_shared<UT::UDP>(
      local_port, target_ip_address.c_str(), target_port, sizeof(UT::HighCmd),
      sizeof(UT::HighState));
    UT::HighCmd init_cmd = {0};
    high_udp_->InitCmdData(init_cmd);

    high_state_pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighState>(
      "high_state", rclcpp::QoS(1));
    cr_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(1));
    cr_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", rclcpp::QoS(1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1));
    high_cmd_sub_ = this->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>(
      "high_cmd", rclcpp::QoS(1),
      std::bind(&UnitreeUDPSender::high_cmd_cb, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(1),
      std::bind(&UnitreeUDPSender::cmd_vel_cb, this, std::placeholders::_1));
    emg_switch_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "emg_switch", std::bind(&UnitreeUDPSender::emg_switch_call, this, std::placeholders::_1, std::placeholders::_2));
    main_timer_ = this->create_wall_timer(2ms, [this]() {
      high_udp_->SetSend(send_cmd_);
      high_udp_->Send();
      ros2_unitree_legged_msgs::msg::HighState state_msg;
      UT::HighState state;
      high_udp_->Recv();
      high_udp_->GetRecv(state);

      nav_msgs::msg::Odometry odom;
      odom.header = make_header(this->get_clock()->now(), "map");
      odom.pose = state2poseWithCovMsg(state);
      odom.twist = state2twistWithCovMsg(state);

      odom_pub_->publish(odom);
    });
  }

  void high_cmd_cb(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
  {
    UT::HighCmd cmd = rosMsg2Cmd(msg);

    high_udp_->SetSend(cmd);
    high_udp_->Send();

    ros2_unitree_legged_msgs::msg::HighState state_msg;
    UT::HighState state;
    high_udp_->Recv();
    high_udp_->GetRecv(state);
    state_msg = state2rosMsg(state);

    high_state_pub_->publish(state_msg);
  }

  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (cr_emg_state_) {
      geometry_msgs::msg::Twist stop_msg;
      stop_msg.linear.x = 0.0;
      stop_msg.linear.y = 0.0;
      stop_msg.linear.z = 0.0;
      stop_msg.angular.x = 0.0;
      stop_msg.angular.y = 0.0;
      stop_msg.angular.z = 0.0;
      send_cmd_ = rosMsg2Cmd(stop_msg);
    } else {
      send_cmd_ = rosMsg2Cmd(*msg);
    }
  }

  void emg_switch_call(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    const std_srvs::srv::SetBool::Response::SharedPtr res)
  {
    cr_emg_state_ = req->data;
    res->success = true;
    res->message = cr_emg_state_ ? "Unitree Go1 cmd_vel control was Deactivated"
                                 : "Unitree Go1 cmd_vel control was Activated";
  }
};
}  // namespace aist_intern2023
