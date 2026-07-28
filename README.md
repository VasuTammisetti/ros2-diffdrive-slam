# ros2-diffdrive-slam

A differential-drive robot I built from scratch in ROS 2 to learn the full 2D SLAM stack, from writing the robot's URDF to driving it around a simulated room and watching slam_toolbox draw the map in real time.

Everything runs in simulation on ROS 2 Jazzy and Gazebo Harmonic. I developed it on Ubuntu 24.04 under WSL2.

## Why I built it

I wanted to actually understand SLAM rather than just run someone else's launch file. So I started with an empty package and added one piece at a time: the robot body, the wheels, a LiDAR, the drive controller, and finally the mapping. Each layer had to work before I moved to the next, which meant I ended up debugging the whole pipeline: TF frames, ros2_control, the Gazebo to ROS sensor bridge, and the SLAM node itself.

## What's in the repo

- A custom differential-drive robot described in URDF/xacro
- A small walled world in Gazebo for the robot to map
- A 360 degree LiDAR publishing laser scans on /scan
- ros2_control with a diff_drive_controller for keyboard driving
- slam_toolbox building and saving a 2D occupancy-grid map
- A saved example map (room_map.pgm and room_map.yaml)

## How it fits together

The robot is spawned into Gazebo, which simulates physics and the LiDAR. robot_state_publisher publishes the robot's TF tree from the URDF. ros2_control, through the gz_ros2_control plugin, runs the controllers: diff_drive_controller turns velocity commands into wheel speeds and publishes wheel odometry plus the odom to base_footprint transform. slam_toolbox takes the laser scans and the TF tree, matches successive scans to build a map, closes loops to correct odometry drift, and publishes the occupancy grid on /map along with the map to odom transform.

The data flow is: teleop publishes cmd_vel, diff_drive_controller converts it to wheel velocities, Gazebo simulates the LiDAR and joint states, robot_state_publisher publishes the TF chain (odom, base_footprint, base_link, wheels, caster, lidar_link), and slam_toolbox consumes the scans and TF to produce the map.

## The robot

A differential-drive base built with xacro macros:

- base_footprint: ground-projected root frame
- base_link: chassis
- left_wheel, right_wheel: continuous joints, actuated by ros2_control
- caster: passive sphere for balance (fixed joint)
- lidar_link: 360 degree laser mount (fixed joint)

The LiDAR is a gpu_lidar sensor: 360 samples over a full circle, 10 Hz, 10 m range, with a bit of Gaussian noise so it's not unrealistically perfect.

## Running it

1. Simulation, robot, and control. The plugin-path export is required, otherwise Gazebo can't find the ros2_control system plugin (see notes below):

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
ros2 launch my_robot gazebo.launch.py
```

2. SLAM:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

3. Drive it. Jazzy's diff_drive_controller expects TwistStamped, hence stamped:=true:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/diff_drive_controller/cmd_vel
```

4. Watch it in RViz. Add Map (/map) and LaserScan (/scan) displays and set Fixed Frame to map.

5. Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/slam_ws/room_map --ros-args -p use_sim_time:=true -p save_map_timeout:=10000.0
```

## Things that bit me (and how I fixed them)

These cost me real time, so I'm writing them down:

- teleop did nothing at first. In Jazzy, diff_drive_controller subscribes to a TwistStamped, not a plain Twist. teleop_twist_keyboard publishes Twist by default, so the robot just sat there. Fix: run teleop with -p stamped:=true.
- The controllers never loaded. The controller_manager service never came up and the spawners timed out. The cause was that Gazebo couldn't find the gz_ros2_control system plugin library. Fix: export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib before launching.
- My world wouldn't load. Passing the world through the ros_gz_sim launch include silently dropped it and Gazebo started an empty default world instead. Running gz sim with the world file as a direct argument fixed it.
- map_saver kept failing. It needs save_map_timeout as a float (10000.0, not 10000) and use_sim_time:=true.

## What I'd do next

- Fuse wheel odometry with an IMU (via robot_localization) so the pose estimate survives wheel slip.
- Add moving obstacles such as simulated people and handle them in the map, which is relevant for robots that share space with humans.
- Layer Nav2 on top of the saved map for autonomous navigation.
- A language layer that turns commands like "go to the corner" into navigation goals.

## Stack

ROS 2 Jazzy, Gazebo Harmonic, ros2_control, slam_toolbox, xacro/URDF, Python launch files, WSL2 (Ubuntu 24.04)
