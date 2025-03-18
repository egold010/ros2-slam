#!/usr/bin/env python3

import os
import random
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Turtlebot3 environment
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    world_launch = os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch)
    )

    # Nav2 stack
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )
    
    # Spawning a box, maybe future goal to find box
    random_x = random.uniform(-5, 5)
    random_y = random.uniform(-5, 5)
    spawn_z = 0.5
    model_path = os.path.join(get_package_share_directory('turtlebot_teleop'), 'models', 'my_box.sdf')
    spawn_box = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-file', model_path,
            '-entity', 'my_box',
            '-x', str(random_x),
            '-y', str(random_y),
            '-z', str(spawn_z)
        ],
        output='screen'
    )

    # SLAM
    slam_self_node = Node(
        package='turtlebot_teleop',
        executable='slam_self',
        name='slam_self',
        output='screen'
    )

    # Navigator
    navigator_node = Node(
        package='turtlebot_teleop',
        executable='navigator',
        name='navigator',
        output='screen'
    )

    # Keyboard teleop, needs to run in separate terminal
    keyboard_teleop_node_in = Node(
        package='turtlebot_teleop',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen'
    )
    keyboard_teleop_node = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'ros2', 'run', 'turtlebot_teleop', 'keyboard_teleop'
        ],
        output='screen'
    )

    # Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        nav2_launch,
        world,
        # spawn_box,
        slam_self_node,
        navigator_node,
        # keyboard_teleop_node,
        rviz_node,
    ])