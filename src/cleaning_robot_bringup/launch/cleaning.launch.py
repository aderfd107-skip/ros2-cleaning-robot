import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("cleaning_robot_bringup")
    default_map = os.path.join(bringup_dir, "maps", "room_map.yaml")

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "nav.launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "world": LaunchConfiguration("world"),
            "world_name": LaunchConfiguration("world_name"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "enable_rviz": LaunchConfiguration("enable_rviz"),
        }.items(),
    )

    coverage_progress = Node(
        package="cleaning_robot_coverage",
        executable="coverage_progress_publisher",
        name="coverage_progress_publisher",
        output="screen",
        parameters=[
            {
                "map_topic": "/map",
                "progress_topic": "/coverage_progress",
                "coverage_map_topic": "/coverage_map",
                "robot_frame": LaunchConfiguration("robot_frame"),
                "coverage_radius": LaunchConfiguration("coverage_radius"),
                "publish_period": LaunchConfiguration("progress_publish_period"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    coverage_planner = Node(
        package="cleaning_robot_coverage",
        executable="coverage_planner",
        name="coverage_planner",
        output="screen",
        parameters=[
            {
                "map_yaml": LaunchConfiguration("map"),
                "use_map_topic": True,
                "map_topic": "/map",
                "line_spacing": LaunchConfiguration("line_spacing"),
                "wall_margin": LaunchConfiguration("wall_margin"),
                "obstacle_margin": LaunchConfiguration("obstacle_margin"),
                "waypoint_spacing": LaunchConfiguration("waypoint_spacing"),
                "turn_margin": LaunchConfiguration("turn_margin"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    cleaning_task_manager = Node(
        package="cleaning_robot_coverage",
        executable="cleaning_task_manager",
        name="cleaning_task_manager",
        output="screen",
        parameters=[
            {
                "map_topic": "/map",
                "waypoints_topic": "/coverage_waypoints",
                "coverage_progress_topic": "/coverage_progress",
                "coverage_target_percent": LaunchConfiguration("coverage_target_percent"),
                "path_wait_timeout": LaunchConfiguration("path_wait_timeout"),
                "single_goal_retry_limit": LaunchConfiguration("single_goal_retry_limit"),
                "max_consecutive_failed_waypoints": LaunchConfiguration(
                    "max_consecutive_failed_waypoints"
                ),
                "skip_failed_waypoints": LaunchConfiguration("skip_failed_waypoints"),
                "recovery_wait_seconds": LaunchConfiguration("recovery_wait_seconds"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Absolute path to the saved occupancy grid yaml file.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="",
                description=(
                    "Absolute path to the Gazebo world file. "
                    "When set, this overrides world_name."
                ),
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="single_obstacle",
                description=(
                    "Preset Gazebo world name passed through nav.launch.py. "
                    "Supported presets: empty_room, single_obstacle, "
                    "multi_obstacle, narrow_passage, multi_room, room."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock.",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Whether to launch RViz.",
            ),
            DeclareLaunchArgument(
                "coverage_target_percent",
                default_value="95.0",
                description="Stop cleaning once this coverage percentage is reached.",
            ),
            DeclareLaunchArgument(
                "path_wait_timeout",
                default_value="30.0",
                description="Seconds to wait for coverage planner output.",
            ),
            DeclareLaunchArgument(
                "single_goal_retry_limit",
                default_value="2",
                description="How many retries are allowed for one waypoint before skipping or failing.",
            ),
            DeclareLaunchArgument(
                "max_consecutive_failed_waypoints",
                default_value="3",
                description="Fail the task after this many skipped waypoints in a row.",
            ),
            DeclareLaunchArgument(
                "skip_failed_waypoints",
                default_value="true",
                description="Skip a waypoint after retries are exhausted.",
            ),
            DeclareLaunchArgument(
                "recovery_wait_seconds",
                default_value="2.0",
                description="Cooldown time before retrying or skipping a failed waypoint.",
            ),
            DeclareLaunchArgument(
                "coverage_radius",
                default_value="0.22",
                description="Coverage marking radius around the robot in meters.",
            ),
            DeclareLaunchArgument(
                "progress_publish_period",
                default_value="1.0",
                description="Coverage progress publish period in seconds.",
            ),
            DeclareLaunchArgument(
                "robot_frame",
                default_value="base_footprint",
                description="Robot base frame used for coverage tracking.",
            ),
            DeclareLaunchArgument(
                "line_spacing",
                default_value="0.24",
                description="Distance between coverage sweep lines in meters.",
            ),
            DeclareLaunchArgument(
                "wall_margin",
                default_value="0.12",
                description="Minimum path clearance from map walls in meters.",
            ),
            DeclareLaunchArgument(
                "obstacle_margin",
                default_value="0.12",
                description="Minimum path clearance from occupied or unknown cells in meters.",
            ),
            DeclareLaunchArgument(
                "waypoint_spacing",
                default_value="0.45",
                description="Approximate distance between generated path waypoints in meters.",
            ),
            DeclareLaunchArgument(
                "turn_margin",
                default_value="0.1",
                description="Inset each sweep line endpoint to leave room for turning.",
            ),
            nav_launch,
            coverage_progress,
            coverage_planner,
            cleaning_task_manager,
        ]
    )
