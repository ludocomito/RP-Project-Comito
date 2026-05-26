# Launches only the viewer; use this with any ROS graph already publishing data.
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = (
        get_package_share_directory("rp_simple_rviz") + "/config/simple_rviz.yaml"
    )

    # Keeping the config as a launch argument makes it easy to test other topic
    # names without editing the installed package.
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="YAML parameter file for simple_rviz_node",
    )

    simple_rviz_node = Node(
        package="rp_simple_rviz",
        executable="simple_rviz_node",
        name="simple_rviz_node",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, simple_rviz_node])
