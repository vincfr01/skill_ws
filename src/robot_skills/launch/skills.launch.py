from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='robot_skills',
            executable='grip_server',
            name='grip_server',
            output='screen'
        ),

        Node(
            package='robot_skills',
            executable='release_server',
            name='release_server',
            output='screen'
        ),

        Node(
            package='robot_skills',
            executable='move_server',
            name='move_server',
            output='screen'
        ),

        Node(
            package='robot_skills',
            executable='pick_server',
            name='pick_server',
            output='screen'
        ),

        Node(
            package='robot_skills',
            executable='place_server',
            name='place_server',
            output='screen'
        ),

        Node(
            package='robot_skills',
            executable='pick_and_place_server',
            name='pick_and_place_server',
            output='screen'
        ),

    ])