from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    map_path = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_velocity_gate = LaunchConfiguration("start_velocity_gate")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Absolute path to the prior PCD map (required)",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=(get_package_share_directory("g1_fast_lio_bringup")
                           + "/config/prior_map_localization.yaml"),
            description="Prior-map localization parameter file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock while replaying a rosbag",
        ),
        DeclareLaunchArgument(
            "start_velocity_gate",
            default_value="false",
            description="Start the fail-closed cmd_vel gate only",
        ),
        Node(
            package="g1_fast_lio_bringup",
            executable="prior_map_localizer",
            name="prior_map_localizer",
            output="screen",
            parameters=[
                params_file,
                {"map_path": map_path, "use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="g1_fast_lio_bringup",
            executable="localization_cmd_vel_gate",
            name="localization_cmd_vel_gate",
            output="screen",
            condition=IfCondition(start_velocity_gate),
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
    ])
