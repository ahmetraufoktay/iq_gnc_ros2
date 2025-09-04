from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument("fcu_url", default_value="udp://127.0.0.1:14551@14555")
    gcs_url_arg = DeclareLaunchArgument("gcs_url", default_value="")
    mavros_ns_arg = DeclareLaunchArgument("mavros_ns", default_value="")
    tgt_system_arg = DeclareLaunchArgument("tgt_system", default_value="1")
    tgt_component_arg = DeclareLaunchArgument("tgt_component", default_value="1")
    pluginlists_yaml_arg = DeclareLaunchArgument("pluginlists_yaml", default_value="")
    config_yaml_arg = DeclareLaunchArgument("config_yaml", default_value="")
    log_output_arg = DeclareLaunchArgument("log_output", default_value="screen")
    fcu_protocol_arg = DeclareLaunchArgument("fcu_protocol", default_value="v2.0")
    respawn_arg = DeclareLaunchArgument("respawn_mavros", default_value="false")

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        namespace=LaunchConfiguration("mavros_ns"),
        output=LaunchConfiguration("log_output"),
        respawn=LaunchConfiguration("respawn_mavros"),
        parameters=[
            {
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": LaunchConfiguration("gcs_url"),
                "system_id": LaunchConfiguration("tgt_system"),
                "component_id": LaunchConfiguration("tgt_component"),
                "fcu_protocol": LaunchConfiguration("fcu_protocol"),
                "conn/timesync_rate": 0.0,
            },
            LaunchConfiguration("pluginlists_yaml"),
            LaunchConfiguration("config_yaml"),
        ],
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        mavros_ns_arg,
        tgt_system_arg,
        tgt_component_arg,
        pluginlists_yaml_arg,
        config_yaml_arg,
        log_output_arg,
        fcu_protocol_arg,
        respawn_arg,
        mavros_node,
    ])
