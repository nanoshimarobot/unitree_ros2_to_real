/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : low_udp(LOWLEVEL, 8007, "192.168.123.161", 8007),
          high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }
};

Custom custom;

rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_high;
rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr sub_low;

rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pub_high;
rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr pub_low;

rclcpp::Node::SharedPtr node;

long high_count = 0;
long low_count = 0;

// Tick synchronization variables
bool tick_initialized = false;
uint32_t initial_tick = 0;
rclcpp::Time initial_ros_time = rclcpp::Time(0);

// Convert tick (ms from motion controller) to ROS time
rclcpp::Time tick_to_ros_time(uint32_t tick)
{
    if (!tick_initialized) {
        // Initialize on first tick
        tick_initialized = true;
        initial_tick = tick;
        initial_ros_time = node->get_clock()->now();
        return initial_ros_time;
    }
    
    // Calculate elapsed time in milliseconds (handle overflow)
    int64_t tick_diff;
    if (tick >= initial_tick) {
        tick_diff = static_cast<int64_t>(tick - initial_tick);
    } else {
        // Handle uint32_t overflow (occurs after ~49.7 days)
        tick_diff = static_cast<int64_t>(static_cast<uint64_t>(tick) + 
                                          (static_cast<uint64_t>(1) << 32) - 
                                          static_cast<uint64_t>(initial_tick));
    }
    
    // Convert milliseconds to nanoseconds and add to initial ROS time
    int64_t elapsed_ns = tick_diff * 1000000LL;  // ms to ns
    rclcpp::Time result_time = initial_ros_time + rclcpp::Duration(0, elapsed_ns);
    
    return result_time;
}

void highCmdCallback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
{
    printf("highCmdCallback is running !\t%ld\n", ::high_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    custom.high_udp.SetSend(custom.high_cmd);
    custom.high_udp.Send();

    ros2_unitree_legged_msgs::msg::HighState high_state_ros;

    custom.high_udp.Recv();
    custom.high_udp.GetRecv(custom.high_state);

    high_state_ros = state2rosMsg(custom.high_state);

    pub_high->publish(high_state_ros);

    printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
}

void lowCmdCallback(const ros2_unitree_legged_msgs::msg::LowCmd::SharedPtr msg)
{
    printf("lowCmdCallback is running !\t%ld\n", low_count);

    custom.low_cmd = rosMsg2Cmd(msg);

    custom.low_udp.SetSend(custom.low_cmd);
    custom.low_udp.Send();

    ros2_unitree_legged_msgs::msg::LowState low_state_ros;

    custom.low_udp.Recv();
    custom.low_udp.GetRecv(custom.low_state);

    // Convert tick to ROS time for accurate timestamp
    rclcpp::Time timestamp = tick_to_ros_time(custom.low_state.tick);
    low_state_ros = state2rosMsg(custom.low_state, timestamp);

    pub_low->publish(low_state_ros);

    printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("node_ros2_udp");

    if (argc < 2)
    {
        std::cout << "Usage: ros2 run unitree_legged_real ros2_udp [highlevel|lowlevel]" << std::endl;
        exit(-1);
    }

    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        printf("low level running!\n");

        pub_low = node->create_publisher<ros2_unitree_legged_msgs::msg::LowState>("low_state", 1);
        sub_low = node->create_subscription<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1, lowCmdCallback);

        rclcpp::spin(node);
    }
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        printf("high level running!\n");

        pub_high = node->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 1);
        sub_high = node->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1, highCmdCallback);

        rclcpp::spin(node);
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel (not case sensitive)" << std::endl;
        exit(-1);
    }

    rclcpp::shutdown();

    return 0;
}
