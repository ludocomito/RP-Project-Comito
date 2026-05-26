# Launches the course simulator and the Simple RViz viewer together.
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("rp_simple_rviz")
    default_config = package_share + "/config/simple_rviz.yaml"
    default_sim_config = package_share + "/assets/diag_single_robot.yaml"

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="YAML parameter file for simple_rviz_node",
    )
    simulator_config_arg = DeclareLaunchArgument(
        "simulator_config",
        default_value=default_sim_config,
        description="YAML world configuration for rp_simulator",
    )
    display_scale_arg = DeclareLaunchArgument(
        "display_scale",
        default_value="0.5",
        description="Scale factor for the OpenCV viewer window",
    )

    simulator_node = Node(
        package="rp_simulator",
        executable="simulator",
        name="simulator_node",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("simulator_config")}],
    )

    simple_rviz_node = Node(
        package="rp_simple_rviz",
        executable="simple_rviz_node",
        name="simple_rviz_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {"display_scale": LaunchConfiguration("display_scale")},
        ],
    )

    return LaunchDescription(
        [
            config_arg,
            simulator_config_arg,
            display_scale_arg,
            simulator_node,
            simple_rviz_node,
        ]
    )
