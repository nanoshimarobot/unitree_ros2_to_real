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
#include "ros2_unitree_legged_msgs/msg/high_cmd_array.hpp"
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
  UT::HighState cr_state_;

  bool cr_emg_state_ = false;

  std::string target_ip_address;
  uint16_t local_port;
  uint16_t target_port;

  std::vector<UT::HighCmd> motion_cmd_;
  size_t motion_execution_cnt_ = 0;

  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr high_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cr_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cr_pos_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr high_cmd_sub_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmdArray>::SharedPtr
    consecutive_motion_sub_;
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

    send_cmd_.euler = {0.f, 0.f, 0.f};
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
    consecutive_motion_sub_ =
      this->create_subscription<ros2_unitree_legged_msgs::msg::HighCmdArray>(
        "consecutive_motion", rclcpp::QoS(1),
        std::bind(&UnitreeUDPSender::consecutive_motion_cb, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(1),
      std::bind(&UnitreeUDPSender::cmd_vel_cb, this, std::placeholders::_1));
    emg_switch_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "emg_switch",
      std::bind(
        &UnitreeUDPSender::emg_switch_call, this, std::placeholders::_1, std::placeholders::_2));
    main_timer_ = this->create_wall_timer(2ms, [this]() {
      if (!motion_cmd_.empty()) {
        high_udp_->SetSend(motion_cmd_[motion_execution_cnt_++]);
        high_udp_->Send();
        if (motion_execution_cnt_ >= motion_cmd_.size()) {
          motion_cmd_.clear();
          motion_execution_cnt_ = 0;
        }
      } else {
        high_udp_->SetSend(send_cmd_);
        high_udp_->Send();
      }
      ros2_unitree_legged_msgs::msg::HighState state_msg;
      UT::HighState state;
      high_udp_->Recv();
      high_udp_->GetRecv(state);
      cr_state_ = state;

      nav_msgs::msg::Odometry odom;
      odom.header = make_header(this->get_clock()->now(), "map");
      odom.pose = state2poseWithCovMsg(state);
      odom.twist = state2twistWithCovMsg(state);

      odom_pub_->publish(odom);
    });
  }

  void high_cmd_cb(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
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
      send_cmd_ = rosMsg2Cmd(msg);
    }
  }

  void consecutive_motion_cb(const ros2_unitree_legged_msgs::msg::HighCmdArray::SharedPtr msg)
  {
    if (msg->high_cmd.empty() || msg->total_execution_time < 1) {
      return;
    }
    // rpy only
    // end pose is necessary
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
      float total_execution_time = msg->total_execution_time;
      int cmd_size_per_motion =
        static_cast<int>(static_cast<int>(total_execution_time / 0.002) / msg->high_cmd.size());
      int total_motion_cmd_size = cmd_size_per_motion * msg->high_cmd.size();
      motion_cmd_.clear();

      std::vector<std::array<float, 3>> motion_base;
      motion_base.push_back(send_cmd_.euler);  // syoki ti
      for (size_t i = 0; i < msg->high_cmd.size(); ++i) {
        motion_base.push_back(msg->high_cmd[i].euler);
      }

      UT::HighCmd cmd;
      cmd =
        rosMsg2Cmd(std::make_shared<ros2_unitree_legged_msgs::msg::HighCmd>(msg->high_cmd.front()));
      for (size_t i = 1; i < motion_base.size(); ++i) {
        auto start = motion_base[i - 1];
        auto end = motion_base[i];
        double r_per_motion =
          static_cast<double>(end[0] - start[0]) / static_cast<double>(cmd_size_per_motion);
        double p_per_motion =
          static_cast<double>(end[1] - start[1]) / static_cast<double>(cmd_size_per_motion);
        double y_per_motion =
          static_cast<double>(end[2] - start[2]) / static_cast<double>(cmd_size_per_motion);
        for (size_t i = 0; i < cmd_size_per_motion; ++i) {
          cmd.euler[0] = static_cast<float>(r_per_motion * static_cast<double>(i));
          cmd.euler[1] = static_cast<float>(p_per_motion * static_cast<double>(i));
          cmd.euler[2] = static_cast<float>(y_per_motion * static_cast<double>(i));
          motion_cmd_.push_back(cmd);
        }
      }
      motion_execution_cnt_ = 0;
    }
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
