# mvsim_nav2_demos

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

      ros2 launch mvsim_nav2_demos turtlebot_simulation_launch.py
      ros2 launch mvsim_nav2_demos greenhouse_simulation_launch.py

      ros2 launch mvsim_nav2_demos greenhouse_simulation_launch.py world_file:=$(pwd)/src/mvsim_nav2_demos/launch/demo_greenhouse_jjaa.world.xml
    
    
This package is based on [nav2_bringup](https://github.com/ros-planning/navigation2/tree/main/nav2_bringup), so we inherit their Apache-2 License here.


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
