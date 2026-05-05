# ROS2 Gazebo Livox Chassis Simulation

This repository provides a ROS2 Humble simulation package for a four-wheel mobile robot chassis in Gazebo Classic and RViz2.

The robot model was originally exported from SolidWorks as URDF, then modified for Gazebo simulation, differential-drive control, RViz visualization, path display, Livox MID-360S-like LiDAR simulation, IMU simulation, GUI control, and keyboard control.

---

## 1. Project Overview

This project includes:

- Four-wheel chassis URDF model
- Gazebo Classic simulation
- RViz2 visualization
- Differential-drive control using `gazebo_ros_diff_drive`
- GUI velocity control
- Keyboard velocity control
- Odometry `/odom`
- TF tree `/tf` and `/tf_static`
- Path display `/path`
- Livox MID-360S-like 360° LiDAR point cloud
- IMU topic `/livox/imu`
- Livox-like point cloud filter `/livox/lidar_like`
- Integrated launch files for Gazebo + RViz2

---

## 2. System Environment

Tested environment:

```text
Ubuntu 22.04
ROS2 Humble
Gazebo Classic 11
RViz2
Python 3
PyQt5
```

Before building or running:

```bash
source /opt/ros/humble/setup.bash
```

---

## 3. Repository Structure

```text
ros2_gazebo_livox_chassis/
├── src/
│   └── chassis_description/
│       ├── launch/
│       │   ├── gazebo_server_only.launch.py
│       │   ├── gazebo_full.launch.py
│       │   ├── gazebo_keyboard_full.launch.py
│       │   └── rviz2_fake_gui.launch.py
│       │
│       ├── meshes/
│       │   ├── base_link.STL
│       │   ├── fixed_items.STL
│       │   ├── front_left.STL
│       │   ├── front_left_wheel.STL
│       │   ├── front_right.STL
│       │   ├── front_right_wheel.STL
│       │   ├── rare_left.STL
│       │   ├── rare_left_wheel.STL
│       │   ├── rare_right.STL
│       │   ├── rare_right_wheel.STL
│       │   ├── lidar_link.STL
│       │   └── ...
│       │
│       ├── urdf/
│       │   ├── chassis.SLDASM.urdf
│       │   ├── rviz2_chassis.SLDASM.urdf
│       │   └── gazebo_chassis_diff.urdf
│       │
│       ├── worlds/
│       │   ├── test_nav_world.world
│       │   └── test_nav_world_visual_only.world
│       │
│       ├── scripts/
│       │   ├── gazebo_gui_drive.py
│       │   ├── gazebo_keyboard_drive.py
│       │   ├── gazebo_joint_state_fix.py
│       │   ├── odom_to_path.py
│       │   ├── livox_like_filter.py
│       │   ├── rviz2_gui_drive.py
│       │   └── rviz2_keyboard_drive.py
│       │
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── README.md
│
└── .gitignore
```

---

## 4. Main URDF Files

### 4.1 Original SolidWorks URDF

```text
src/chassis_description/urdf/chassis.SLDASM.urdf
```

This is the original URDF exported from SolidWorks. It is mainly kept as a reference and backup.

### 4.2 RViz2-only URDF

```text
src/chassis_description/urdf/rviz2_chassis.SLDASM.urdf
```

This version is used for RViz2-only visualization and fake motion testing. It does not include Gazebo physics or Gazebo plugins.

### 4.3 Gazebo Simulation URDF

```text
src/chassis_description/urdf/gazebo_chassis_diff.urdf
```

This is the main URDF used for Gazebo simulation. It includes:

- Gazebo differential-drive plugin
- Wheel friction settings
- Gazebo materials
- Livox MID-360S-like LiDAR
- IMU simulation
- Standard scan frame `livox_scan_link`

---

## 5. Differential Drive Configuration

The robot is controlled by:

```xml
<plugin name="gazebo_ros_diff_drive" filename="libgazebo_ros_diff_drive.so">
```

Main configuration:

```xml
<left_joint>front_left_wheel_joint</left_joint>
<right_joint>front_right_wheel_joint</right_joint>

<wheel_separation>0.55</wheel_separation>
<wheel_diameter>0.21</wheel_diameter>

<max_wheel_torque>500.0</max_wheel_torque>
<max_wheel_acceleration>50.0</max_wheel_acceleration>

<command_topic>cmd_vel</command_topic>

<publish_odom>true</publish_odom>
<publish_odom_tf>true</publish_odom_tf>
<publish_wheel_tf>false</publish_wheel_tf>

<odometry_frame>odom</odometry_frame>
<robot_base_frame>base_footprint</robot_base_frame>
```

The robot subscribes to `/cmd_vel` and publishes `/odom` and `/tf`.

---

## 6. Robot Forward Direction

The robot has been adjusted so that:

```text
The Livox LiDAR side is the forward direction of the robot.
```

Therefore:

```text
linear.x > 0  -> move forward toward the LiDAR side
linear.x < 0  -> move backward
```

---

## 7. LiDAR Simulation

### 7.1 Original Simulated LiDAR Topic

The Gazebo LiDAR publishes:

```text
/livox/lidar
```

Message type:

```text
sensor_msgs/msg/PointCloud2
```

The LiDAR is mounted on a dedicated scan frame:

```text
livox_scan_link
```

This scan frame was added because the original SolidWorks `lidar_link` orientation was not suitable for Gazebo ray scanning.

Current structure:

```text
lidar_link        -> visual LiDAR mesh
livox_scan_link   -> Gazebo LiDAR scan frame
```

This makes the raw LiDAR scan correctly cover 360 degrees.

### 7.2 Livox-like Filtered Point Cloud

The package also includes a Livox-like visualization filter:

```text
scripts/livox_like_filter.py
```

Input:

```text
/livox/lidar
```

Output:

```text
/livox/lidar_like
```

Output fields:

```text
x
y
z
intensity
```

The goal of this filter is to make the Gazebo point cloud visually closer to a Livox-style point cloud by:

- Keeping 360° coverage
- Slightly breaking the regular Gazebo ray pattern
- Adding an `intensity` field
- Adding light texture and jitter
- Preserving all directions so obstacles are not lost

Important note:

```text
This is a Livox-like visualization filter, not an official physical Livox MID-360S simulator.
```

Gazebo Classic `gpu_ray` cannot perfectly reproduce the real Livox non-repetitive scanning mechanism.

---

## 8. IMU Simulation

The robot includes a simulated IMU topic:

```text
/livox/imu
```

Message type:

```text
sensor_msgs/msg/Imu
```

The IMU is attached to:

```text
imu_link
```

---

## 9. Main ROS2 Topics

After launching the simulation, common topics include:

```text
/clock
/cmd_vel
/joint_states
/odom
/path
/robot_description
/tf
/tf_static
/livox/lidar
/livox/lidar_like
/livox/imu
/rosout
```

Topic descriptions:

```text
/cmd_vel           velocity command
/odom              robot odometry
/tf                dynamic transforms
/tf_static         static transforms
/joint_states      wheel joint states for RViz visualization
/path              odometry path generated from /odom
/livox/lidar       original Gazebo 360° point cloud
/livox/lidar_like  Livox-like filtered point cloud
/livox/imu         simulated IMU data
```

---

## 10. Build Instructions

From the workspace root:

```bash
cd ~/chassis
colcon build --packages-select chassis_description
source install/setup.bash
```

If this repository is cloned to another directory, replace `~/chassis` with your workspace path.

---

## 11. Launch Files

### 11.1 Full Gazebo + RViz2 + GUI Control

Recommended launch:

```bash
cd ~/chassis
source install/setup.bash
ros2 launch chassis_description gazebo_full.launch.py
```

This starts:

```text
gzserver
gzclient
rviz2
gazebo_joint_state_fix.py
odom_to_path.py
livox_like_filter.py
gazebo_gui_drive.py
```

### 11.2 Gazebo + RViz2 + Keyboard Control

```bash
cd ~/chassis
source install/setup.bash
ros2 launch chassis_description gazebo_keyboard_full.launch.py
```

Keyboard controls:

```text
i        forward
m / s    backward
j        turn left
l        turn right
k        stop
Space    stop
q / z    increase / decrease both speed and turn rate
w / x    increase / decrease linear speed
e / c    increase / decrease angular speed
```

### 11.3 RViz2-only Fake Motion Mode

```bash
cd ~/chassis
source install/setup.bash
ros2 launch chassis_description rviz2_fake_gui.launch.py
```

This mode is used only for RViz2 visualization and fake motion testing. It does not use Gazebo physics.

---

## 12. GUI Control

The Gazebo GUI control script is:

```text
scripts/gazebo_gui_drive.py
```

It publishes only:

```text
/cmd_vel
```

It does not publish `/tf`, `/odom`, or `/joint_states`.

Controls:

```text
Forward
Backward
Turn left
Turn right
Stop
```

Keyboard shortcuts inside the GUI:

```text
i        forward
m / s    backward
j        turn left
l        turn right
k        stop
Space    stop
```

---

## 13. Path Display

The script:

```text
scripts/odom_to_path.py
```

subscribes to `/odom` and publishes `/path`.

In RViz2:

```text
Add -> Path
Topic -> /path
```

---

## 14. RViz2 Settings

Recommended RViz2 settings:

```text
Fixed Frame = odom
```

RobotModel:

```text
Description Source = Topic
Description Topic = /robot_description
```

Point cloud:

```text
Add -> PointCloud2
Topic = /livox/lidar_like
Style = Flat Squares
Size (m) = 0.015 or 0.02
```

For Livox-style green visualization:

```text
Color Transformer = FlatColor
Color = 0, 255, 80
```

For intensity visualization:

```text
Color Transformer = Intensity
Channel Name = intensity
Use rainbow = false
```

---

## 15. Check TF

List TF topics:

```bash
ros2 topic list | grep tf
```

Echo transform between frames:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_footprint livox_scan_link
```

Generate TF tree:

```bash
sudo apt install ros-humble-tf2-tools
ros2 run tf2_tools view_frames
xdg-open frames.pdf
```

---

## 16. Check LiDAR Topics

List Livox topics:

```bash
ros2 topic list | grep livox
```

Expected output:

```text
/livox/imu
/livox/lidar
/livox/lidar_like
```

Check frequency:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/lidar_like
```

Check topic type:

```bash
ros2 topic info /livox/lidar
ros2 topic info /livox/lidar_like
```

---

## 17. World Files

World files are stored in:

```text
src/chassis_description/worlds/
```

Example world:

```text
test_nav_world.world
```

The world contains:

- Ground plane
- Walls
- Box obstacles
- Goal marker

The world is loaded by:

```text
launch/gazebo_server_only.launch.py
```

To switch back to an empty world, modify the world path in that launch file:

```python
"/usr/share/gazebo-11/worlds/empty.world"
```

---

## 18. Common Problems

### 18.1 RViz2 cannot find `chassis_description`

Solution:

```bash
source ~/chassis/install/setup.bash
```

Then check:

```bash
ros2 pkg prefix chassis_description
```

### 18.2 RViz2 model does not display

Use:

```text
RobotModel -> Description Source = Topic
RobotModel -> Description Topic = /robot_description
```

Do not use an old URDF file path if Gazebo is running.

### 18.3 Wheel TF becomes red in RViz2

Check that `/joint_states` names match URDF joint names:

```text
front_right_wheel_joint
front_left_wheel_joint
rare_left_wheel_joint
rare_right_wheel_joint
```

### 18.4 LiDAR is not 360°

Check that the point cloud is coming from:

```text
livox_scan_link
```

and not directly from the SolidWorks visual `lidar_link`.

Also check:

```bash
ros2 run tf2_ros tf2_echo base_footprint livox_scan_link
```

### 18.5 `/livox/lidar_like` loses obstacles

Use the stable 360° version of `livox_like_filter.py`.

The filter should not use a strong rose or petal pattern that removes entire directions.

### 18.6 `/cmd_vel` works but robot direction is wrong

Check the signs in:

```text
scripts/gazebo_gui_drive.py
scripts/gazebo_keyboard_drive.py
```

For the current setup:

```text
linear.x > 0 -> forward toward the LiDAR side
linear.x < 0 -> backward
```

---

## 19. Development Notes

This simulation is intended to verify:

- ROS2 topic connections
- TF tree correctness
- Gazebo and RViz integration
- Robot velocity control
- Odometry and path tracking
- LiDAR point cloud visualization
- Sensor frame alignment
- Basic navigation platform setup

It is not intended to perfectly reproduce the physical behavior of a real Livox MID-360S LiDAR.

Recommended technical description:

```text
The Gazebo LiDAR model approximates the Livox MID-360S field of view and point cloud output for system-level navigation and visualization tests. Since Gazebo Classic gpu_ray does not support Livox's real non-repetitive scanning mechanism, an additional Livox-like point cloud filter is provided for visualization.
```

---

## 20. Future Work

Possible future extensions:

- Nav2 integration
- PointCloud2 to LaserScan conversion
- SLAM mapping
- Costmap generation
- Goal-point navigation
- Obstacle avoidance
- Agricultural field simulation world
- Integration with real Livox MID-360S
- Integration with 3D Gaussian Splatting localization
- Real robot deployment

---

## 21. Quick Start

```bash
cd ~/chassis
colcon build --packages-select chassis_description
source install/setup.bash
ros2 launch chassis_description gazebo_full.launch.py
```

Then in RViz2:

```text
Fixed Frame = odom
PointCloud2 Topic = /livox/lidar_like
Path Topic = /path
RobotModel Description Topic = /robot_description
```

Use the GUI window to drive the robot.
