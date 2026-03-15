from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


OVERRIDABLE_ARGUMENTS = (
    (
        "can",
        "Optional SocketCAN interface override, for example can0.",
    ),
    (
        "topic_name",
        "Optional odometry topic override, for example /odom.",
    ),
    (
        "frame_id",
        "Optional odometry frame override, for example odom.",
    ),
    (
        "child_frame_id",
        "Optional child frame override, for example base_link.",
    ),
    (
        "txt_path",
        "Optional TXT output directory override, or an explicit .txt file path.",
    ),
    (
        "txt_name",
        "Optional TXT output file name override. Use auto to keep timestamp naming.",
    ),
    (
        "txt_name_format",
        "Optional strftime format override used when txt_name is auto.",
    ),
    (
        "txt_is",
        "Optional TXT logging switch override: on/off.",
    ),
    (
        "txt_format",
        "Optional TXT logging format override: can or odom_csv.",
    ),
    (
        "socket_timeout_sec",
        "Optional CAN socket timeout override in seconds.",
    ),
)


def launch_setup(context, *args, **kwargs):
    overrides = {}
    for name, _description in OVERRIDABLE_ARGUMENTS:
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            if name == "socket_timeout_sec":
                overrides[name] = float(value)
            else:
                overrides[name] = value

    parameters = [LaunchConfiguration("params_file").perform(context)]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package="gi5651_can_odom",
            executable="gi5651_can_odom_node",
            name="gi5651_can_odom",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("gi5651_can_odom"), "config", "can_2_txt.yaml"]
    )

    launch_arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to the ROS 2 parameter YAML file.",
        )
    ]
    launch_arguments.extend(
        DeclareLaunchArgument(name, default_value="", description=description)
        for name, description in OVERRIDABLE_ARGUMENTS
    )
    launch_arguments.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_arguments)
