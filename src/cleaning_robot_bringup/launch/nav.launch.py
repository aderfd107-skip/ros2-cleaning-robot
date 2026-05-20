import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("cleaning_robot_bringup")
    simulation_dir = get_package_share_directory("cleaning_robot_simulation")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    nav2_params = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    rviz_config = os.path.join(bringup_dir, "config", "nav.rviz")
    default_map = os.path.join(bringup_dir, "maps", "room_map.yaml")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_dir, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "world_name": LaunchConfiguration("world_name"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "spawn_delay": LaunchConfiguration("spawn_delay"),
            "gazebo_gui": LaunchConfiguration("enable_gazebo_gui"),
        }.items(),
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Absolute path to the saved occupancy grid yaml file.",
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value="",
        description=(
            "Absolute path to the Gazebo world file. "
            "When set, this overrides world_name."
        ),
    )

    declare_world_name = DeclareLaunchArgument(
        "world_name",
        default_value="single_obstacle",
        description=(
            "Preset Gazebo world name passed to sim.launch.py. "
            "Supported presets: empty_room, single_obstacle, "
            "multi_obstacle, narrow_passage, multi_room, room."
        ),
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock.",
    )

    declare_enable_rviz = DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
        description="Whether to launch RViz for navigation validation.",
    )

    declare_enable_gazebo_gui = DeclareLaunchArgument(
        "enable_gazebo_gui",
        default_value=LaunchConfiguration("enable_rviz"),
        description=(
            "Whether to launch Gazebo GUI (gzclient). "
            "Defaults to the same value as enable_rviz."
        ),
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config,
        description="Absolute path to the RViz config file.",
    )

    declare_spawn_x = DeclareLaunchArgument(
        "spawn_x",
        default_value="0.0",
        description="Robot spawn x position in Gazebo.",
    )

    declare_spawn_y = DeclareLaunchArgument(
        "spawn_y",
        default_value="0.0",
        description="Robot spawn y position in Gazebo.",
    )

    declare_spawn_yaw = DeclareLaunchArgument(
        "spawn_yaw",
        default_value="0.0",
        description="Robot spawn yaw in radians.",
    )

    declare_spawn_z = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.05",
        description="Robot spawn z position in Gazebo.",
    )

    declare_spawn_delay = DeclareLaunchArgument(
        "spawn_delay",
        default_value="2.0",
        description="Delay before spawning the robot into Gazebo.",
    )

    declare_initial_pose_delay = DeclareLaunchArgument(
        "initial_pose_delay",
        default_value="4.0",
        description="Delay before publishing the AMCL initial pose.",
    )

    declare_initial_pose_max_attempts = DeclareLaunchArgument(
        "initial_pose_max_attempts",
        default_value="30",
        description="Maximum number of /initialpose messages to publish before stopping.",
    )

    declare_initial_pose_xy_tolerance = DeclareLaunchArgument(
        "initial_pose_xy_tolerance",
        default_value="0.20",
        description="Maximum XY error in meters before AMCL is considered aligned.",
    )

    declare_initial_pose_yaw_tolerance = DeclareLaunchArgument(
        "initial_pose_yaw_tolerance",
        default_value="0.35",
        description="Maximum yaw error in radians before AMCL is considered aligned.",
    )

    declare_initial_pose_stable_count = DeclareLaunchArgument(
        "initial_pose_stable_count",
        default_value="3",
        description="Number of consecutive aligned /amcl_pose samples required before stopping.",
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "true",
            "params_file": nav2_params,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        output="screen",
    )

    initial_pose_publisher = Node(
        package="cleaning_robot_bringup",
        executable="publish_initial_pose.py",
        name="initial_pose_publisher",
        output="screen",
        parameters=[
            {
                "x": LaunchConfiguration("spawn_x"),
                "y": LaunchConfiguration("spawn_y"),
                "yaw": LaunchConfiguration("spawn_yaw"),
                "publish_delay": LaunchConfiguration("initial_pose_delay"),
                "max_attempts": LaunchConfiguration("initial_pose_max_attempts"),
                "xy_tolerance": LaunchConfiguration("initial_pose_xy_tolerance"),
                "yaw_tolerance": LaunchConfiguration("initial_pose_yaw_tolerance"),
                "required_stable_count": LaunchConfiguration(
                    "initial_pose_stable_count"
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            declare_map,
            declare_world,
            declare_world_name,
            declare_use_sim_time,
            declare_enable_rviz,
            declare_enable_gazebo_gui,
            declare_rviz_config,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_yaw,
            declare_spawn_z,
            declare_spawn_delay,
            declare_initial_pose_delay,
            declare_initial_pose_max_attempts,
            declare_initial_pose_xy_tolerance,
            declare_initial_pose_yaw_tolerance,
            declare_initial_pose_stable_count,
            simulation,
            bringup,
            initial_pose_publisher,
            rviz,
        ]
    )
