# Introduction

This package can send control command to real robot from ROS2. You can do low-level control (namely control all joints on robot) and high-level control (namely control the walking direction and speed of robot).

This version is suitable for unitree_legged_sdk v3.5.1, namely Go1 robot.

**✨ Enhanced Features:**
- **Low State Support**: Read detailed robot state including joint angles, torques, and sensor data via `/low_state` topic at 500Hz
- **Timestamp Synchronization**: LowState messages include ROS Header with timestamps based on Go1's motion controller `tick` field
- **Self-contained Repository**: Message definitions (`ros2_unitree_legged_msgs`) are included in this repository
- **Two Execution Modes**: Original style (`ros2_udp`) and enhanced component-based node (`unitree_udp_sender`)

## Packages

**Basic message function**: `ros2_unitree_legged_msgs`
- Includes 12 message types: BmsCmd, BmsState, Cartesian, HighCmd, HighCmdArray, HighState, IMU, LED, LowCmd, LowState, MotorCmd, MotorState
- LowState now includes `std_msgs/Header` for timestamp information

**The interface between ROS and real robot**: `unitree_legged_real`
- Original style: `ros2_udp` executable (command-driven communication)
- Enhanced style: `unitree_udp_sender` component (timer-driven continuous communication with additional features)

## Environment

We recommend users to run this package in:
- Ubuntu 18.04 / 20.04 / 22.04 / 24.04
- ROS2 Eloquent / Foxy / Galactic / Humble / Jazzy

## Dependencies

* [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk): v3.5.1

## Dependencies

* [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk): v3.5.1

# Configuration

First, create a directory.

```bash
mkdir -p ~/ros2_ws/src
```

Then download this package into this `~/ros2_ws/src` folder.

```bash
cd ~/ros2_ws/src
git clone <this_repository_url> unitree_ros2_to_real
```

After you download this package into this folder, your folder should be like this:

```bash
~/ros2_ws/src/unitree_ros2_to_real
```

**Important**: Create a symbolic link for the message package so that colcon can detect it:

```bash
cd ~/ros2_ws/src
ln -s unitree_ros2_to_real/ros2_unitree_legged_msgs ros2_unitree_legged_msgs
```

This symbolic link is necessary because colcon only detects packages directly under the `src` directory.

And now download unitree_legged_sdk v3.5.1 into the path `~/ros2_ws/src/unitree_ros2_to_real`:

```bash
cd ~/ros2_ws/src/unitree_ros2_to_real
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git
```

# Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

After building, source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

You should see two packages built successfully:
- `ros2_unitree_legged_msgs`
- `unitree_legged_real`

# Setup the net connection

First, please connect the network cable between your PC and robot. Then run `ifconfig` in a terminal, you will find your port name. For example, `enx000ec6612921`.

Then, you can set up the network manually:

```bash
sudo ifconfig <your_port_name> 192.168.123.162 netmask 255.255.255.0
```

For example:
```bash
sudo ifconfig enx000ec6612921 192.168.123.162 netmask 255.255.255.0
```

If you want to set your port automatically on startup, you can modify `interfaces`:

```bash
sudo gedit /etc/network/interfaces
```

And add the following lines at the end (on Ubuntu 18.04 / 20.04):

```
auto enx000ec6612921
iface enx000ec6612921 inet static
address 192.168.123.162
netmask 255.255.255.0
```

Where the port name has to be changed to your own.

**Note**: On Ubuntu 22.04 and later, you may need to use Netplan instead:

```bash
sudo nano /etc/netplan/01-unitree-network.yaml
```

Add:
```yaml
network:
  version: 2
  ethernets:
    enx000ec6612921:  # Change to your interface name
      addresses:
        - 192.168.123.162/24
      dhcp4: false
```

Apply the configuration:
```bash
sudo netplan apply
```

You can verify the connection by pinging the robot:

```bash
ping 192.168.123.161
```

# Run the package

## Method 1: Original Style (ros2_udp)

Before you do high level or low level control, you should run the `ros2_udp` node, which is a bridge that connects users and robot.

For **low level** mode:

```bash
ros2 run unitree_legged_real ros2_udp lowlevel
```

For **high level** mode:

```bash
ros2 run unitree_legged_real ros2_udp highlevel
```

It depends which control mode (low level or high level) you want to use.

### In High Level Mode

In the high level mode, you can control the robot by publishing to `/high_cmd` topic:

```bash
ros2 topic pub /high_cmd ros2_unitree_legged_msgs/msg/HighCmd "{...}"
```

The robot state will be published to `/high_state` topic.

### In Low Level Mode

In the low level mode, you can control the robot by publishing to `/low_cmd` topic:

```bash
ros2 topic pub /low_cmd ros2_unitree_legged_msgs/msg/LowCmd "{...}"
```

The robot state will be published to `/low_state` topic.

**✨ Enhanced Feature**: The `/low_state` topic now includes:
- `std_msgs/Header header`: Timestamp synchronized with Go1's motion controller `tick` field
- All 20 joint states (angle, velocity, torque, temperature)
- IMU data (quaternion, gyroscope, accelerometer, RPY)
- Battery management system (BMS) data
- Foot force sensors
- Wireless remote input

And before you do the low-level control, please press `L2+A` to sit the robot down and then press `L1+L2+start` to make the robot into mode in which you can do joint-level control. Finally, make sure you hang the robot up before you run low-level control.

## Method 2: Enhanced Component Node (unitree_udp_sender)

This is an enhanced version that continuously reads robot state at 500Hz and publishes to multiple topics:

```bash
ros2 run unitree_legged_real unitree_udp_sender
```

This node provides:
- Continuous state updates (both High State and Low State at 500Hz)
- `/cmd_vel` interface for easy robot control
- Odometry (`/odom`), pose (`/pose`), and velocity (`/velocity`) topics
- Emergency stop service (`/emg_switch`)

### Controlling the robot with cmd_vel

```bash
# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Or use keyboard control:

```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# Monitoring Robot State

You can monitor various aspects of the robot state:

```bash
# List all topics
ros2 topic list

# View Low State data
ros2 topic echo /low_state

# Check update rate (should be around 500Hz)
ros2 topic hz /low_state

# View specific joint data (e.g., joint 0 angle)
ros2 topic echo /low_state/motor_state[0]/q

# View IMU data
ros2 topic echo /low_state/imu

# View battery information
ros2 topic echo /low_state/bms

# View timestamp and tick
ros2 topic echo /low_state --field header.stamp --field tick
```

# Timestamp Synchronization

The Low State messages now include a `std_msgs/Header` with timestamp information:

- **`header.stamp`**: ROS timestamp synchronized with Go1's motion controller
- **`tick`**: Raw tick value from Go1 (milliseconds since motion controller startup)

The timestamp is generated as follows:
1. On first message: Record initial `tick` and corresponding PC time
2. On subsequent messages: Calculate elapsed time from tick difference and add to initial PC time
3. Handles uint32_t overflow automatically (occurs after ~49.7 days)

This ensures that the relative timing between samples is accurate to Go1's internal clock, while being compatible with ROS2 time systems.

# Topics Published by unitree_udp_sender

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/high_state` | `ros2_unitree_legged_msgs/HighState` | 500Hz | High-level robot state |
| `/low_state` | `ros2_unitree_legged_msgs/LowState` | 500Hz | Low-level robot state with timestamp |
| `/odom` | `nav_msgs/Odometry` | 500Hz | Odometry information |
| `/pose` | `geometry_msgs/PoseStamped` | 500Hz | Robot pose |
| `/velocity` | `geometry_msgs/TwistStamped` | 500Hz | Robot velocity |

# Troubleshooting

## Build errors

If you encounter build errors related to `ros2_unitree_legged_msgs`:

```bash
# Make sure the symbolic link is created
ls -la ~/ros2_ws/src/ros2_unitree_legged_msgs

# Rebuild messages first
cd ~/ros2_ws
colcon build --packages-select ros2_unitree_legged_msgs
source install/setup.bash
colcon build --packages-select unitree_legged_real
```

## Cannot connect to robot

1. Check network connection: `ping 192.168.123.161`
2. Check network interface IP: `ifconfig`
3. Check firewall: `sudo ufw status`
4. Temporarily disable firewall for testing: `sudo ufw disable`

## Low update rate

If topic update rate is much lower than 500Hz:
1. Check CPU usage: `top`
2. Use USB 3.0 Ethernet adapter (not USB 2.0)
3. Increase process priority: `sudo nice -n -20 ros2 run ...`

# License

TODO: Add license information

# Maintainer

unitree <laikago@unitree.cc>

---

**Safety Notice**: Always ensure there are no people or obstacles within the robot's range of motion before starting control. Keep the emergency stop button within reach at all times.
