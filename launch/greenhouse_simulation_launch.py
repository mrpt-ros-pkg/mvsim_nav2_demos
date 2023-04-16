# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
#from ament_index_python import get_package_share_directory
import os

mvsimDir = get_package_share_directory("mvsim")
mvsimNav2DemoDir = get_package_share_directory("mvsim_nav2_demos")

MVSIM_WORLD_FILE = os.path.join(mvsimDir, 'mvsim_tutorial',
                                'demo_greenhouse.world.xml')
MVSIM_ROS2_PARAMS_FILE = os.path.join(mvsimDir, 'mvsim_tutorial',
                                      'mvsim_ros2_params.yaml')
RVIZ2_FILE = os.path.join(mvsimNav2DemoDir, 'rviz',
                          'greenhouse.rviz')


def generate_launch_description():
    # Get the launch directory
    launch_dir = os.path.join(mvsimNav2DemoDir, 'launch')

    # args that can be set from the command line or a default will be used
    world_file_launch_arg = DeclareLaunchArgument(
        "world_file", default_value=TextSubstitution(
            text=MVSIM_WORLD_FILE))

    # Create the launch configuration variables
    params_file = LaunchConfiguration('params_file')  # for nav2
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    headless = LaunchConfiguration('headless')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            mvsimNav2DemoDir, 'params', 'greenhouse_nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo/MVsim) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_use_slam = DeclareLaunchArgument(
        'use_slam', default_value='False',
        description='Use SLAM.')

    # Specify the actions
    # start_gazebo_server_cmd = ExecuteProcess(
    # condition=IfCondition(use_simulator),
    # cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
    # '-s', 'libgazebo_ros_factory.so', world],
    # cwd=[launch_dir], output='screen')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'mvsim_nav2_bringup_launch.py')),
        launch_arguments={
            'slam': use_slam,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn}.items())

    # 'log_level': 'debug',

    # Nodes:
    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[
            MVSIM_ROS2_PARAMS_FILE,
            {
                "world_file": LaunchConfiguration('world_file'),
                "headless": False
            }]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
                '-d', [RVIZ2_FILE]]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_entity(world_file_launch_arg)
    ld.add_entity(mvsim_node)
    ld.add_entity(rviz2_node)

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_slam)

    ld.add_action(bringup_cmd)

    return ld
