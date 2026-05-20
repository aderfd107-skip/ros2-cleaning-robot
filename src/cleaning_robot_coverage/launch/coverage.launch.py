import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map = ""
    try:
        bringup_dir = get_package_share_directory("cleaning_robot_bringup")
        default_map = os.path.join(bringup_dir, "maps", "room_map.yaml")
    except PackageNotFoundError:
        pass

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_yaml",
                default_value=default_map,
                description="Absolute path to an occupancy grid yaml file.",
            ),
            DeclareLaunchArgument(
                "use_map_topic",
                default_value="false",
                description="Use /map OccupancyGrid instead of loading map_yaml.",
            ),
            DeclareLaunchArgument(
                "line_spacing",
                default_value="0.30",
                description="Distance between sweep lines in meters.",
            ),
            DeclareLaunchArgument(
                "wall_margin",
                default_value="0.20",
                description="Minimum path clearance from map walls in meters.",
            ),
            DeclareLaunchArgument(
                "obstacle_margin",
                default_value="0.20",
                description="Minimum path clearance from occupied or unknown cells in meters.",
            ),
            DeclareLaunchArgument(
                "waypoint_spacing",
                default_value="0.30",
                description="Approximate distance between generated path waypoints in meters.",
            ),
            DeclareLaunchArgument(
                "turn_margin",
                default_value="0.25",
                description="Inset each sweep line endpoint to leave room for turning.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock.",
            ),
            Node(
                package="cleaning_robot_coverage",
                executable="coverage_planner",
                name="coverage_planner",
                output="screen",
                parameters=[
                    {
                        "map_yaml": LaunchConfiguration("map_yaml"),
                        "use_map_topic": LaunchConfiguration("use_map_topic"),
                        "line_spacing": LaunchConfiguration("line_spacing"),
                        "wall_margin": LaunchConfiguration("wall_margin"),
                        "obstacle_margin": LaunchConfiguration("obstacle_margin"),
                        "waypoint_spacing": LaunchConfiguration("waypoint_spacing"),
                        "turn_margin": LaunchConfiguration("turn_margin"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
        ]
    )
