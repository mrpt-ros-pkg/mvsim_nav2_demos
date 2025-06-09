# ROS2 launch file

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    mvsim_dir = get_package_share_directory('mvsim')
    mvsim_tutorial_dir = os.path.join(mvsim_dir, 'mvsim_tutorial')

    mvsim_nav2_demo_dir = get_package_share_directory('mvsim_nav2_demos')
    mvsim_nav2_launch_dir = os.path.join(mvsim_nav2_demo_dir, 'launch')

    ### mvsim
    world_file_launch_arg = DeclareLaunchArgument(
        "world_file", default_value=TextSubstitution(
        text=os.path.join(mvsim_nav2_launch_dir, 'demo_warehouse_6robots.world.xml')))

    headless_launch_arg = DeclareLaunchArgument(
        "headless", default_value='False')

    do_fake_localization_arg = DeclareLaunchArgument(
        "do_fake_localization", default_value='True', description='publish tf odom -> base_link')

    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[
            os.path.join(mvsim_tutorial_dir,
                         'mvsim_ros2_params.yaml'),
            {
                "world_file": LaunchConfiguration('world_file'),
                "headless": LaunchConfiguration('headless'),
                "do_fake_localization": LaunchConfiguration('do_fake_localization'),
            }]
    )

    ### rviz
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')

    rviz2_nodes = [
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=f"veh{idx}",
            arguments=[
                '-d', [os.path.join(mvsim_tutorial_dir, 'demo_warehouse_6robots_vehs_ros2.rviz')]],
            output='screen',
            remappings=[
                ("/map", "map"),
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
                ("/goal_pose", "goal_pose"),
                ("/clicked_point", "clicked_point"),
                ("/vehs/cmd_vel", f"/veh{idx}/cmd_vel"),
                ("/vehs/lidar1_points", f"/veh{idx}/lidar1_points"),
                ("/vehs/cam1", f"/veh{idx}/cam1"),
                ("/vehs/chassis_markers", f"/veh{idx}/chassis_markers"),
                ("/vehs/scanner1", f"/veh{idx}/scanner1"),
                ("/vehs/simul_map", f"/veh{idx}/simul_map"),
                ("/vehs/simul_map_updates", f"/veh{idx}/simul_map_updates"),
            ],
        )
        for idx in range(1, 7)
    ]

    return LaunchDescription(
        [
            world_file_launch_arg,
            headless_launch_arg,
            do_fake_localization_arg,
            mvsim_node,
            declare_use_rviz_cmd,
        ]
        + rviz2_nodes
    )
