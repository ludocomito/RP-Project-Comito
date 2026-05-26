# Launches the map server and the Simple RViz viewer together.
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("rp_simple_rviz")
    default_config = package_share + "/config/simple_rviz.yaml"
    default_map = package_share + "/assets/diag.png"

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="YAML parameter file for simple_rviz_node",
    )
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=default_map,
        description="Path to a PNG map image",
    )
    resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="0.1",
        description="Map resolution in meters per pixel",
    )
    origin_x_arg = DeclareLaunchArgument(
        "origin_x",
        default_value="0.0",
        description="World x coordinate of the map origin",
    )
    origin_y_arg = DeclareLaunchArgument(
        "origin_y",
        default_value="-98.7",
        description="World y coordinate of the map origin",
    )
    display_scale_arg = DeclareLaunchArgument(
        "display_scale",
        default_value="0.5",
        description="Scale factor for the OpenCV viewer window",
    )

    map_server_node = Node(
        package="rp_simple_rviz",
        executable="map_server_node",
        name="simple_map_server",
        output="screen",
        parameters=[
            {
                "map_path": LaunchConfiguration("map_path"),
                "resolution": ParameterValue(
                    LaunchConfiguration("resolution"), value_type=float
                ),
                "origin_x": ParameterValue(
                    LaunchConfiguration("origin_x"), value_type=float
                ),
                "origin_y": ParameterValue(
                    LaunchConfiguration("origin_y"), value_type=float
                ),
            }
        ],
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
            map_path_arg,
            resolution_arg,
            origin_x_arg,
            origin_y_arg,
            display_scale_arg,
            map_server_node,
            simple_rviz_node,
        ]
    )
