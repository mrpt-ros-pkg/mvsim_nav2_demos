# mvsim_nav2_demos

## Launch with Nav2

Launch and configuration files for running [Nav2](https://navigation.ros.org/) on [MVsim](https://github.com/MRPT/mvsim) worlds using ROS 2.

Usage:

- Clone into your `workspace/src` directory.
- Install dependencies (TESTED ON ROS humble !!):

  ```bash
  sudo apt install \
    ros-$ROS_DISTRO-mvsim \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-turtlebot3*
  ```

- Build as usual (colcon build).

  ```bash
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  . install/setup.bash
  ```

- Invoke with:

  ```bash
  ros2 launch mvsim_nav2_demos turtlebot_simulation_launch.py
  ros2 launch mvsim_nav2_demos greenhouse_simulation_launch.py

  ros2 launch mvsim_nav2_demos greenhouse_simulation_launch.py world_file:=$(pwd)/src/mvsim_nav2_demos/launch/demo_greenhouse_jjaa.world.xml
  ```

This package is based on [nav2_bringup](https://github.com/ros-planning/navigation2/tree/main/nav2_bringup), so we inherit their Apache-2 License here.

### Multi-agent demo

Multi-agent demo supports multiple agents with fake-amcl (ground truth on mvsim).
Therefore, this demo is useful for verification and implementation of the overall system, including path planning.

- Invoke mvsim with:

  ```bash
  ros2 launch mvsim_nav2_demos demo_warehouse_6robots.launch.py
  ```

- Invoke nav2 with:

  ```bash
  ros2 launch mvsim_nav2_demos mvsim_vehs_launch.py namespace:=veh1
  ros2 launch mvsim_nav2_demos mvsim_vehs_launch.py namespace:=veh2
  ros2 launch mvsim_nav2_demos mvsim_vehs_launch.py namespace:=veh3
  ros2 launch mvsim_nav2_demos mvsim_vehs_launch.py namespace:=veh4
  ros2 launch mvsim_nav2_demos mvsim_vehs_launch.py namespace:=veh5
  ros2 launch mvsim_nav2_demos mvsim_vehs_launch.py namespace:=veh6
  ```

#### How to send a goal

- Send goal via rviz:

  - Click the ***Nav2 Goal*** button on the top bar.
  - Click and hold on the goal position and drag to the goal heading.
  - Then you can see a green arrow and that is the goal (2d position and heading).

- Send goal action via terminal:

  - The namespace is ***veh1*** and ***-f*** option is for the feedback information

  ```bash
  ros2 action send_goal /veh1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"map\"}, \
  pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, \
  behavior_tree: \"\"}" -f
  ```

- Send goal poses action via terminal:

  - The namespace is ***veh1*** and ***-f*** option is for the feedback information

  ```bash
  ros2 action send_goal /veh1/navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
    "{poses: [
    {header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"map\"}, \
    pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, \
    {header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"map\"}, \
    pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, \
    {header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"map\"}, \
    pose: {position: {x: 5.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, \
    ], \
    behavior_tree: \"\"}" -f
  ```

## Launch alternative with mrpt_navigation

- Clone into "src" https://github.com/mrpt-ros-pkg/mrpt_navigation.git
- Build:

  ```bash
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  . install/setup.bash
  ```

- Launch:

  ```bash
  ros2 launch mrpt_tutorials demo_reactive_nav_mvsim.launch.py  world_file:=$(pwd)/src/mvsim_nav2_demos/launch/demo_greenhouse_jjaa.world.xml
  ```
