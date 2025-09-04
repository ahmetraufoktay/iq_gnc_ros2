from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="udp://127.0.0.1:14551@14555"
    )
    gcs_url_arg = DeclareLaunchArgument(
        "gcs_url",
        default_value=""
    )
    tgt_system_arg = DeclareLaunchArgument(
        "tgt_system",
        default_value="1"
    )
    tgt_component_arg = DeclareLaunchArgument(
        "tgt_component",
        default_value="1"
    )
    log_output_arg = DeclareLaunchArgument(
        "log_output",
        default_value="screen"
    )
    respawn_arg = DeclareLaunchArgument(
        "respawn_mavros",
        default_value="true"
    )
    mavros_ns_arg = DeclareLaunchArgument(
        "mavros_ns",
        default_value="/"
    )
    config_yaml_arg = DeclareLaunchArgument(
        "config_yaml",
        default_value=PathJoinSubstitution([
            FindPackageShare("mavros"),
            "launch",
            "apm_config.yaml"
        ])
    )

    mavros_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("iq_gnc_ros2"),
                "launch",
                "mavros_node.launch.py"
            ])
        ),
        launch_arguments={
            "pluginlists_yaml": PathJoinSubstitution([
                FindPackageShare("mavros"),
                "launch",
                "apm_pluginlists.yaml"
            ]),
            "config_yaml": LaunchConfiguration("config_yaml"),
            "mavros_ns": LaunchConfiguration("mavros_ns"),
            "fcu_url": LaunchConfiguration("fcu_url"),
            "gcs_url": LaunchConfiguration("gcs_url"),
            "respawn_mavros": LaunchConfiguration("respawn_mavros"),
            "tgt_system": LaunchConfiguration("tgt_system"),
            "tgt_component": LaunchConfiguration("tgt_component"),
            "log_output": LaunchConfiguration("log_output"),
        }.items()
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        log_output_arg,
        respawn_arg,
        mavros_ns_arg,
        config_yaml_arg,
        mavros_node_launch
    ])
