import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_auto_explore_params(config_path):
    if not config_path or not os.path.isfile(config_path):
        return {}

    with open(config_path, "r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}

    node_block = data.get("auto_explore_and_save", {})
    params = node_block.get("ros__parameters", {})
    if not isinstance(params, dict):
        return {}
    return params


def _resolve_profile_name(world_override_path: str, world_name: str) -> str:
    profile_name = world_name
    if world_override_path:
        profile_name = os.path.splitext(os.path.basename(world_override_path))[0] or world_name
    return profile_name


def _create_auto_explore_action(context, bringup_dir):
    base_config = LaunchConfiguration("auto_explore_config").perform(context)
    world_override_path = LaunchConfiguration("world").perform(context).strip()
    world_name = LaunchConfiguration("world_name").perform(context).strip()
    exploration_timeout_override = (
        LaunchConfiguration("exploration_timeout").perform(context).strip()
    )
    workspace_dir = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(bringup_dir)))
    )
    profile_name = _resolve_profile_name(world_override_path, world_name)

    world_override_candidates = [
        os.path.join(bringup_dir, "config", "auto_explore", f"{profile_name}.yaml"),
        os.path.join(
            workspace_dir,
            "src",
            "cleaning_robot_bringup",
            "config",
            "auto_explore",
            f"{profile_name}.yaml",
        ),
    ]

    world_profile_path = ""
    merged_params = _load_auto_explore_params(base_config)
    for candidate in world_override_candidates:
        if os.path.isfile(candidate):
            world_profile_path = candidate
            break

    merged_params.update(
        {
            "use_sim_time": LaunchConfiguration("use_sim_time").perform(context).lower()
            == "true",
            "world_name": world_name,
            "world_profile_path": world_profile_path,
            "save_map_url": LaunchConfiguration("map_output").perform(context),
        }
    )
    if exploration_timeout_override:
        merged_params["exploration_timeout"] = float(exploration_timeout_override)

    return [
        TimerAction(
            period=float(LaunchConfiguration("auto_explore_start_delay").perform(context)),
            actions=[
                Node(
                    package="cleaning_robot_bringup",
                    executable="auto_explore_and_save.py",
                    parameters=[merged_params],
                    output="screen",
                )
            ],
        )
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory("cleaning_robot_bringup")
    simulation_dir = get_package_share_directory("cleaning_robot_simulation")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    workspace_dir = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(bringup_dir)))
    )

    slam_config = os.path.join(bringup_dir, "config", "slam_toolbox.yaml")
    auto_explore_config = os.path.join(bringup_dir, "config", "auto_explore.yaml")
    nav2_params = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    rviz_config = os.path.join(bringup_dir, "config", "slam.rviz")
    default_map_output = os.path.join(
        workspace_dir, "src", "cleaning_robot_bringup", "maps", "room_map"
    )

    declare_map_output = DeclareLaunchArgument(
        "map_output",
        default_value=default_map_output,
        description="Absolute base path where the generated map files will be saved.",
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

    declare_exploration_timeout = DeclareLaunchArgument(
        "exploration_timeout",
        default_value="",
        description=(
            "Optional soft exploration timeout override in seconds. "
            "Leave empty to use the base or world-specific yaml value."
        ),
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock for all nodes started by this launch file.",
    )

    declare_enable_rviz = DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
        description="Whether to launch RViz for SLAM visualization.",
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

    declare_auto_explore_config = DeclareLaunchArgument(
        "auto_explore_config",
        default_value=auto_explore_config,
        description=(
            "Absolute path to the base auto exploration parameter yaml file. "
            "If config/auto_explore/<world_name>.yaml exists, it will be applied on top."
        ),
    )

    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=nav2_params,
        description="Absolute path to the Nav2 parameters yaml used during SLAM exploration.",
    )

    declare_auto_explore_start_delay = DeclareLaunchArgument(
        "auto_explore_start_delay",
        default_value="8.0",
        description="Delay in seconds before the auto exploration node starts.",
    )

    declare_map_saver_start_delay = DeclareLaunchArgument(
        "map_saver_start_delay",
        default_value="1.0",
        description="Delay in seconds before starting the map_saver lifecycle manager.",
    )

    declare_entity_name = DeclareLaunchArgument(
        "entity_name",
        default_value="cleaning_robot",
        description="Gazebo entity name for the spawned robot.",
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

    declare_spawn_z = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.05",
        description="Robot spawn z position in Gazebo.",
    )

    declare_spawn_yaw = DeclareLaunchArgument(
        "spawn_yaw",
        default_value="0.0",
        description="Robot spawn yaw in radians.",
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_dir, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "world_name": LaunchConfiguration("world_name"),
            "entity_name": LaunchConfiguration("entity_name"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "gazebo_gui": LaunchConfiguration("enable_gazebo_gui"),
        }.items(),
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[
            slam_config,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "true",
            "params_file": LaunchConfiguration("nav2_params_file"),
            "use_composition": "False",
            "use_respawn": "False",
            "container_name": "nav2_container",
            "log_level": "info",
        }.items(),
    )

    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "save_map_timeout": 5.0,
                "free_thresh_default": 0.25,
                "occupied_thresh_default": 0.65,
            }
        ],
        output="screen",
    )

    lifecycle_manager = TimerAction(
        period=LaunchConfiguration("map_saver_start_delay"),
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_slam",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "autostart": True,
                        "node_names": ["map_saver"],
                    }
                ],
                output="screen",
            )
        ],
    )

    auto_explore = OpaqueFunction(
        function=_create_auto_explore_action,
        args=[bringup_dir],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            declare_map_output,
            declare_world,
            declare_world_name,
            declare_exploration_timeout,
            declare_use_sim_time,
            declare_enable_rviz,
            declare_enable_gazebo_gui,
            declare_rviz_config,
            declare_auto_explore_config,
            declare_nav2_params,
            declare_auto_explore_start_delay,
            declare_map_saver_start_delay,
            declare_entity_name,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            declare_spawn_yaw,
            simulation,
            slam,
            navigation,
            map_saver,
            lifecycle_manager,
            auto_explore,
            rviz,
        ]
    )
