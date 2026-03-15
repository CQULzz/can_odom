from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


OVERRIDABLE_ARGUMENTS = (
    ("can", "Optional SocketCAN interface override, for example can0."),
    ("txt_path", "Optional TXT input file override. This must point to a .txt file."),
    ("send_interval_sec", "Optional fixed delay between CAN frames in seconds."),
    ("loop", "Optional replay loop switch override: on/off."),
)


def launch_setup(context, *args, **kwargs):
    overrides = {}
    for name, _description in OVERRIDABLE_ARGUMENTS:
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            if name == "send_interval_sec":
                overrides[name] = float(value)
            else:
                overrides[name] = value

    parameters = [LaunchConfiguration("params_file").perform(context)]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package="gi5651_can_odom",
            executable="gi5651_txt_2_can_node",
            name="gi5651_txt_2_can",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("gi5651_can_odom"), "config", "txt_2can.yaml"]
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
