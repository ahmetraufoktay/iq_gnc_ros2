from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value="true",
            description="Use simulation (Gazebo) clock if true"
        ),        
        Node(
            package="iq_gnc",
            executable="yolo_node.py",
            output="screen"
        )
    ])
