from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    drones = ["drone1", "drone2", "drone3", "drone4"]

    nodes = []
    for drone in drones:
        nodes.append(
            Node(
                package="iq_gnc_ros2",
                executable="square",
                name=f"{drone}_square",
                namespace=f"/{drone}",
                output="screen",
                parameters=[{
                    "namespace": f"/{drone}",
                    "use_sim_time": True,
                }],
            )
        )

    return LaunchDescription(nodes)
