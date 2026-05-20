import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_PRESETS = {
    "empty_room": "empty_room.world",
    "single_obstacle": "single_obstacle.world",
    "multi_obstacle": "multi_obstacle.world",
    "narrow_passage": "narrow_passage.world",
    "multi_room": "multi_room.world",
    "room": "room.world",
}

def _resolve_world_file(context, simulation_dir):
    world_override = LaunchConfiguration("world").perform(context).strip()
    if world_override:
        world_file = world_override
    else:
        world_name = LaunchConfiguration("world_name").perform(context).strip()
        preset_file = WORLD_PRESETS.get(world_name)
        if preset_file is None:
            supported_worlds = ", ".join(sorted(WORLD_PRESETS))
            raise RuntimeError(
                f"Unknown world_name '{world_name}'. "
                f"Supported presets: {supported_worlds}."
            )
        world_file = os.path.join(simulation_dir, "worlds", preset_file)

    if not os.path.exists(world_file):
        raise RuntimeError(f"Gazebo world file does not exist: {world_file}")

    return world_file


def _create_gazebo_launch(context, gazebo_ros_dir, simulation_dir):
    world_file = _resolve_world_file(context, simulation_dir)
    gazebo_gui = LaunchConfiguration("gazebo_gui").perform(context).strip()
    gazebo_server = LaunchConfiguration("gazebo_server").perform(context).strip()
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")
            ),
            launch_arguments={
                "world": world_file,
                "gui": gazebo_gui,
                "server": gazebo_server,
            }.items(),
        )
    ]
def generate_launch_description():
    description_dir = get_package_share_directory("cleaning_robot_description")
    simulation_dir = get_package_share_directory("cleaning_robot_simulation")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    urdf_file = os.path.join(description_dir, "urdf", "robot.urdf.xacro")
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
            "Preset Gazebo world name. Supported presets: "
            "empty_room, single_obstacle, multi_obstacle, "
            "narrow_passage, multi_room, room."
        ),
    )

    declare_entity_name = DeclareLaunchArgument(
        "entity_name",
        default_value="cleaning_robot",
        description="Gazebo entity name for the robot.",
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

    declare_spawn_delay = DeclareLaunchArgument(
        "spawn_delay",
        default_value="2.0",
        description="Delay in seconds before spawning the robot into Gazebo.",
    )

    declare_gazebo_gui = DeclareLaunchArgument(
        "gazebo_gui",
        default_value="true",
        description='Whether to launch gzclient. Set to "false" for headless simulation.',
    )

    declare_gazebo_server = DeclareLaunchArgument(
        "gazebo_server",
        default_value="true",
        description='Whether to launch gzserver.',
    )

    robot_description = ParameterValue(
        Command(["xacro", " ", urdf_file]),
        value_type=str,
    )

    gazebo = OpaqueFunction(
        function=_create_gazebo_launch,
        args=[gazebo_ros_dir, simulation_dir],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
                "rate": 30,
                "publish_default_positions": True,
            }
        ],
        output="screen",
    )

    spawn_robot = TimerAction(
        period=LaunchConfiguration("spawn_delay"),
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-entity",
                    LaunchConfiguration("entity_name"),
                    "-x",
                    LaunchConfiguration("spawn_x"),
                    "-y",
                    LaunchConfiguration("spawn_y"),
                    "-z",
                    LaunchConfiguration("spawn_z"),
                    "-Y",
                    LaunchConfiguration("spawn_yaw"),
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            declare_world,
            declare_world_name,
            declare_entity_name,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            declare_spawn_yaw,
            declare_spawn_delay,
            declare_gazebo_gui,
            declare_gazebo_server,
            gazebo,
            robot_state_publisher,
            joint_state_publisher,
            spawn_robot,
        ]
    )
