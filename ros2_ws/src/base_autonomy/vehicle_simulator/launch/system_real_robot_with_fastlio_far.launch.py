import os
import fcntl
from xml.sax.saxutils import quoteattr

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitution import Substitution
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


class _BridgeDdsConfig(Substitution):
    """Configure the one shared ROS/SDK Cyclone domain before either starts."""

    def perform(self, context):
        interface = LaunchConfiguration("network_interface").perform(context)
        localhost = os.environ.get("ROS_LOCALHOST_ONLY", "0") == "1"
        if localhost and interface != "lo":
            raise RuntimeError("A localhost-only motion bridge requires network_interface:=lo")
        # rmw_cyclonedds adds lo itself when ROS_LOCALHOST_ONLY is set.
        interfaces = "" if localhost else (
            "<Interfaces><NetworkInterface name=" + quoteattr(interface) + "/></Interfaces>"
        )
        discovery = (
            "<Discovery><ParticipantIndex>auto</ParticipantIndex>"
            "<MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>"
            "<Peers><Peer address=\"127.0.0.1\"/></Peers></Discovery>"
        ) if interface == "lo" else ""
        multicast = "<AllowMulticast>false</AllowMulticast>" if interface == "lo" else ""
        return "<CycloneDDS><Domain id=\"any\"><General>" + interfaces + multicast + "</General>" + discovery + "</Domain></CycloneDDS>"


def _is_value(configuration, expected):
    return IfCondition(
        PythonExpression(["'", configuration, "' == '", expected, "'"])
    )


def _is_not_value(configuration, expected):
    return IfCondition(
        PythonExpression(["'", configuration, "' != '", expected, "'"])
    )


def _is_one_of(configuration, *expected_values):
    values = ", ".join(repr(value) for value in expected_values)
    return IfCondition(
        PythonExpression(["'", configuration, "' in (", values, ")"])
    )


def _validate_arguments(context):
    operation = LaunchConfiguration("operation").perform(context)
    planner_mode = LaunchConfiguration("planner_mode").perform(context)
    prior_map = LaunchConfiguration("map").perform(context)
    enable_motion = LaunchConfiguration("enable_motion").perform(context).lower()
    network_interface = LaunchConfiguration("network_interface").perform(context)
    inverted = LaunchConfiguration("lidar_inverted").perform(context).lower()
    # Both operations support inverted mounting: mapping_frame_adapter creates
    # Z-up assets, while prior_map_localizer aligns to existing Z-up assets.
    if inverted not in ("true", "false"):
        raise RuntimeError("lidar_inverted must be 'true' or 'false'")

    if operation not in ("mapping", "localization"):
        raise RuntimeError("operation must be 'mapping' or 'localization'")
    if planner_mode not in ("far", "manual", "none"):
        raise RuntimeError(
            "planner_mode must be 'far', 'manual' or 'none'"
        )
    if operation == "localization" and not prior_map:
        raise RuntimeError("map:=/absolute/path/to/prior.pcd is required in localization mode")
    if operation == "localization" and not os.path.isabs(prior_map):
        raise RuntimeError("map must be an absolute PCD path")
    if enable_motion not in ("true", "false"):
        raise RuntimeError("enable_motion must be 'true' or 'false'")
    if enable_motion == "true" and LaunchConfiguration("use_sim_time").perform(context).lower() != "false":
        raise RuntimeError("Physical motion requires use_sim_time:=false")
    if enable_motion == "true" and planner_mode not in ("far",):
        raise RuntimeError(
            "enable_motion:=true requires planner_mode:=far; "
            "manual mode deliberately leaves motion to the native G1 controller"
        )
    if enable_motion == "true" and not os.path.exists(
        os.path.join("/sys/class/net", network_interface)
    ):
        raise RuntimeError(f"network interface does not exist: {network_interface}")
    # Held by this LaunchContext until launch exits, including direct invocations.
    lock = open("/tmp/g1_fastlio_far_launch.lock", "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError("Another FAST-LIO/FAR launch is running")
    context._fastlio_far_lock = lock
    return []


def generate_launch_description():
    lidar_inverted = LaunchConfiguration("lidar_inverted")
    initial_heading_search = LaunchConfiguration("initial_heading_search")
    operation = LaunchConfiguration("operation")
    planner_mode = LaunchConfiguration("planner_mode")
    route_planner_config = LaunchConfiguration("route_planner_config")
    world_name = LaunchConfiguration("world_name")
    robot_config = LaunchConfiguration("robot_config")
    autonomy_mode = LaunchConfiguration("autonomyMode")
    camera_offset_z = LaunchConfiguration("cameraOffsetZ")
    vehicle_x = LaunchConfiguration("vehicleX")
    vehicle_y = LaunchConfiguration("vehicleY")
    check_terrain_conn = LaunchConfiguration("checkTerrainConn")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_mid360_driver = LaunchConfiguration("start_mid360_driver")
    start_fastlio = LaunchConfiguration("start_fastlio")
    start_joy = LaunchConfiguration("start_joy")
    enable_visualization_tools = LaunchConfiguration("start_visualization_tools")
    fastlio_params = LaunchConfiguration("fastlio_params")
    localization_params = LaunchConfiguration("localization_params")
    prior_map = LaunchConfiguration("map")
    enable_motion = LaunchConfiguration("enable_motion")
    network_interface = LaunchConfiguration("network_interface")
    g1_loco_client_path = LaunchConfiguration("g1_loco_client_path")
    unitree_sdk_lib_path = LaunchConfiguration("unitree_sdk_lib_path")
    max_x_speed = LaunchConfiguration("max_x_speed")
    max_y_speed = LaunchConfiguration("max_y_speed")
    max_angular_speed = LaunchConfiguration("max_angular_speed")
    use_final_goal_pose = LaunchConfiguration("useFinalGoalPose")
    goal_require_localization_ready = LaunchConfiguration("goalRequireLocalizationReady")
    goal_position_tolerance = LaunchConfiguration("goalPositionTolerance")
    goal_alignment_exit_frames = LaunchConfiguration("goalAlignmentExitFrames")
    goal_alignment_exit_distance = LaunchConfiguration("goalAlignmentExitDistance")
    goal_yaw_tolerance = LaunchConfiguration("goalYawTolerance")
    goal_confirm_frames = LaunchConfiguration("goalConfirmFrames")
    goal_stopped_linear_speed = LaunchConfiguration("goalStoppedLinearSpeed")
    goal_stopped_angular_speed = LaunchConfiguration("goalStoppedAngularSpeed")

    fastlio_share = get_package_share_directory("g1_fast_lio_bringup")

    declared_arguments = [
        DeclareLaunchArgument("lidar_inverted", default_value="false", choices=["true", "false"],
                              description="Upside-down MID360; mapping saves Z-up assets, localization uses a Z-up PCD/VGH pair"),
        DeclareLaunchArgument("initial_heading_search", default_value="false", choices=["true", "false"],
                              description="At localization initialization compare seed yaw and yaw+pi; reject ambiguous matches"),
        DeclareLaunchArgument(
            "operation",
            default_value="mapping",
            description="mapping: make a new PCD/VGH pair; localization: use a prior PCD",
        ),
        DeclareLaunchArgument(
            "planner_mode",
            default_value="far",
            description=(
                "far: RViz goals; manual: native-controller mapping/localization "
                "with a passive FAR graph and no ROS motion chain; none: "
                "localization only for an external Nav2 stack"
            ),
        ),
        DeclareLaunchArgument("map", default_value="", description="Absolute prior PCD path"),
        DeclareLaunchArgument("route_planner_config", default_value="indoor"),
        DeclareLaunchArgument("world_name", default_value="real_world"),
        DeclareLaunchArgument("robot_config", default_value="unitree/unitree_g1"),
        DeclareLaunchArgument("autonomyMode", default_value="true"),
        DeclareLaunchArgument("cameraOffsetZ", default_value="0.25"),
        DeclareLaunchArgument("vehicleX", default_value="0.0"),
        DeclareLaunchArgument("vehicleY", default_value="0.0"),
        DeclareLaunchArgument("checkTerrainConn", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_mid360_driver", default_value="true"),
        DeclareLaunchArgument("start_fastlio", default_value="true"),
        DeclareLaunchArgument("start_joy", default_value="true"),
        DeclareLaunchArgument(
            "start_visualization_tools",
            default_value="true",
            description=(
                "Start the legacy accumulated-map visualizer. Disable during "
                "prior-map localization to avoid loading/publishing a duplicate PCD."
            ),
        ),
        DeclareLaunchArgument(
            "enable_motion",
            default_value="false",
            description="Explicit opt-in for the isolated SDK2 G1 motion backend",
        ),
        DeclareLaunchArgument("network_interface", default_value="eno1"),
        DeclareLaunchArgument(
            "g1_loco_client_path",
            default_value=EnvironmentVariable("G1_LOCO_CLIENT_BIN", default_value=""),
        ),
        DeclareLaunchArgument(
            "unitree_sdk_lib_path",
            default_value=EnvironmentVariable("UNITREE_SDK_LIB_DIR", default_value=""),
        ),
        DeclareLaunchArgument("max_x_speed", default_value="0.20"),
        DeclareLaunchArgument("max_y_speed", default_value="0.10"),
        DeclareLaunchArgument("max_angular_speed", default_value="0.35"),
        DeclareLaunchArgument("useFinalGoalPose", default_value="true"),
        DeclareLaunchArgument("goalRequireLocalizationReady", default_value="false"),
        DeclareLaunchArgument("goalPositionTolerance", default_value="0.10"),
        DeclareLaunchArgument("goalAlignmentExitFrames", default_value="3"),
        DeclareLaunchArgument("goalAlignmentExitDistance", default_value="0.30"),
        DeclareLaunchArgument("goalYawTolerance", default_value="0.3490658503988659"),
        DeclareLaunchArgument("goalConfirmFrames", default_value="3"),
        DeclareLaunchArgument("goalStoppedLinearSpeed", default_value="0.03"),
        DeclareLaunchArgument("goalStoppedAngularSpeed", default_value="0.05"),
        DeclareLaunchArgument(
            "fastlio_params",
            default_value=os.path.join(
                fastlio_share, "config", "g1_mid360_localization.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "localization_params",
            default_value=os.path.join(
                fastlio_share, "config", "prior_map_localization.yaml"
            ),
        ),
    ]

    start_mid360 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("livox_ros_driver2"),
                "launch_ROS2",
                "msg_MID360_launch.py",
            )
        ),
        condition=IfCondition(start_mid360_driver),
    )

    # Mapping owns map -> camera_init via the adapter. It rotates both point
    # coordinates and navigation odometry; a TF-only flip is insufficient for
    # consumers which use message coordinates directly.
    mapping_frame_adapter = Node(
        package="g1_fast_lio_bringup",
        executable="mapping_frame_adapter",
        name="mapping_frame_adapter",
        output="screen",
        parameters=[{
            "lidar_inverted": ParameterValue(lidar_inverted, value_type=bool),
            "navigation_frame": PythonExpression(["'sensor' if '", lidar_inverted, "' == 'true' else 'body'"]),
            "use_sim_time": use_sim_time,
        }],
        condition=_is_value(operation, "mapping"),
    )

    # FAST-LIO odometry describes its IMU/body origin. The legacy autonomy
    # stack names that same odometry origin sensor, then applies the configured
    # sensor mounting offsets in sensor -> vehicle. Connect the two trees once;
    # map -> camera_init remains owned by the localizer (or mapping_frame_adapter).
    fastlio_sensor_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fastlio_body_to_sensor",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", PythonExpression(["'3.141592653589793' if '", lidar_inverted, "' == 'true' else '0'"]),
            "--pitch", "0", "--yaw", "0",
            "--frame-id", "body", "--child-frame-id", "sensor",
        ],
        condition=_is_not_value(planner_mode, "none"),
    )

    fastlio_mapping = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[fastlio_params, {"use_sim_time": use_sim_time}],
        remappings=[
            # FAST-LIO's Path is an odometry/trajectory visualization path,
            # not the localPlanner navigation path consumed by pathFollower.
            ("/path", "/fastlio_path"),
        ],
        condition=IfCondition(
            PythonExpression([
                "'", operation, "' == 'mapping' and '", start_fastlio, "' == 'true'"
            ])
        ),
    )

    fastlio_localization = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[fastlio_params, {"use_sim_time": use_sim_time}],
        remappings=[("/path", "/fastlio_path")],
        condition=IfCondition(
            PythonExpression([
                "'", operation, "' == 'localization' and '", start_fastlio, "' == 'true'"
            ])
        ),
    )

    start_prior_localizer = Node(
        package="g1_fast_lio_bringup",
        executable="prior_map_localizer",
        name="prior_map_localizer",
        output="screen",
        parameters=[
            localization_params,
            {"map_path": prior_map, "use_sim_time": use_sim_time,
             "lidar_inverted": ParameterValue(lidar_inverted, value_type=bool),
             "initial_heading_search": ParameterValue(initial_heading_search, value_type=bool),
             "navigation_frame": PythonExpression(["'sensor' if '", lidar_inverted, "' == 'true' else 'body'"])},
        ],
        condition=_is_value(operation, "localization"),
    )

    # Keep this stack's velocity away from the shared /cmd_vel bus. Other
    # navigation launches are therefore unaffected by the dedicated gate.
    start_local_planner = GroupAction(
        actions=[
            SetRemap(src="/cmd_vel", dst="/cmd_vel_fastlio_far_raw"),
            SetRemap(src="/path", dst="/fastlio_far/local_path"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("local_planner"),
                        "launch",
                        "local_planner.launch.py",
                    )
                ),
                launch_arguments={
                    "robot_config": robot_config,
                    "autonomyMode": autonomy_mode,
                    "requireGoal": "true",
                    # FAR/TARE publish PointStamped waypoints without yaw.
                    "useGoalYaw": "false",
                    "useFinalGoalPose": use_final_goal_pose,
                    "goalRequireLocalizationReady": PythonExpression([
                        "'", operation, "' == 'localization' or '",
                        goal_require_localization_ready, "' == 'true'"
                    ]),
                    "goalPositionTolerance": goal_position_tolerance,
                    "goalAlignmentExitFrames": goal_alignment_exit_frames,
                    "goalAlignmentExitDistance": goal_alignment_exit_distance,
                    "goalYawTolerance": goal_yaw_tolerance,
                    "goalConfirmFrames": goal_confirm_frames,
                    "goalStoppedLinearSpeed": goal_stopped_linear_speed,
                    "goalStoppedAngularSpeed": goal_stopped_angular_speed,
                    "cameraOffsetZ": camera_offset_z,
                    "goalX": vehicle_x,
                    "goalY": vehicle_y,
                }.items(),
            ),
        ],
        condition=_is_value(planner_mode, "far"),
    )

    motion_gate = Node(
        package="vehicle_simulator",
        executable="fastlio_far_motion_gate",
        name="fastlio_far_motion_gate",
        output="screen",
        parameters=[{
            "input_topic": "/cmd_vel_fastlio_far_raw",
            "path_topic": "/fastlio_far/local_path",
            "output_topic": "/cmd_vel_fastlio_far_safe",
            "enable_motion": ParameterValue(enable_motion, value_type=bool),
            "require_localization_ready": ParameterValue(
                PythonExpression(["'", operation, "' == 'localization'"]),
                value_type=bool,
            ),
            "max_x_speed": ParameterValue(max_x_speed, value_type=float),
            "max_y_speed": ParameterValue(max_y_speed, value_type=float),
            "max_angular_speed": ParameterValue(max_angular_speed, value_type=float),
        }],
        condition=_is_value(planner_mode, "far"),
    )

    # This is the only node in this launch allowed to reach the G1. It is
    # started only after the operator explicitly passes enable_motion:=true.
    sdk2_motion_backend = Node(
        package="g1_controller",
        executable="cmd_vel_to_g1_persistent",
        name="fastlio_far_cmd_vel_to_g1",
        output="screen",
        # SDK2's libddscxx and libddsc must come from the same distribution.
        # The ELF RUNPATH loses to ROS's LD_LIBRARY_PATH; override only this
        # process so the rest of the ROS workflow keeps its normal libraries.
        additional_env={
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "CYCLONEDDS_URI": _BridgeDdsConfig(),
            "LD_LIBRARY_PATH": [
                unitree_sdk_lib_path, os.pathsep,
                EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
            ],
        },
        parameters=[{
            "network_interface": network_interface,
            "g1_loco_client_path": g1_loco_client_path,
            "unitree_sdk_lib_path": unitree_sdk_lib_path,
            "command_timeout": 0.2,
            "max_x_linear_velocity": ParameterValue(max_x_speed, value_type=float),
            "max_y_linear_velocity": ParameterValue(max_y_speed, value_type=float),
            "max_angular_velocity": ParameterValue(max_angular_speed, value_type=float),
        }],
        remappings=[("cmd_vel", "/cmd_vel_fastlio_far_safe")],
        condition=IfCondition(
            PythonExpression([
                "'", enable_motion, "' == 'true' and '", planner_mode,
                "' == 'far'"
            ])
        ),
    )

    start_terrain_analysis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("terrain_analysis"),
                "launch",
                "terrain_analysis.launch.py",
            )
        ),
        launch_arguments={"robot_config": robot_config}.items(),
        condition=_is_not_value(planner_mode, "none"),
    )

    start_terrain_analysis_ext = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("terrain_analysis_ext"),
                "launch",
                "terrain_analysis_ext.launch",
            )
        ),
        launch_arguments={"checkTerrainConn": check_terrain_conn}.items(),
        condition=_is_not_value(planner_mode, "none"),
    )

    start_sensor_scan_generation = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sensor_scan_generation"),
                "launch",
                "sensor_scan_generation.launch",
            )
        ),
        condition=_is_not_value(planner_mode, "none"),
    )

    start_visualization_tools = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("visualization_tools"),
                "launch",
                "visualization_tools.launch",
            )
        ),
        launch_arguments={"world_name": world_name}.items(),
        condition=IfCondition(
            PythonExpression([
                "'", planner_mode, "' != 'none' and '",
                enable_visualization_tools, "' == 'true'",
            ])
        ),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="ps3_joy",
        output="screen",
        parameters=[{"dev": "/dev/input/js0", "deadzone": 0.12, "autorepeat_rate": 0.0}],
        condition=IfCondition(
            PythonExpression([
                "'", start_joy, "' == 'true' and '", planner_mode,
                "' == 'far'"
            ])
        ),
    )

    far_launch_source = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("far_planner"), "launch", "far_planner.launch"
        )
    )

    start_far_navigation = IncludeLaunchDescription(
        far_launch_source,
        launch_arguments={"config": route_planner_config, "use_sim_time": use_sim_time}.items(),
        condition=_is_value(planner_mode, "far"),
    )

    # Manual mode keeps a passive FAR graph for visualization while the native
    # controller remains the only motion owner.
    start_passive_far_graph = GroupAction(
        actions=[
            SetRemap(src="/goal_point", dst="/far_graph_only/goal_point"),
            SetRemap(src="/goal_pose", dst="/far_graph_only/goal_pose"),
            SetRemap(src="/way_point", dst="/far_graph_only/way_point"),
            SetRemap(src="/joy", dst="/far_graph_only/joy"),
            IncludeLaunchDescription(
                far_launch_source,
                launch_arguments={"config": route_planner_config, "use_sim_time": use_sim_time}.items(),
            ),
        ],
        condition=_is_value(planner_mode, "manual"),
    )


    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=_validate_arguments),
            SetParameter(name="use_sim_time", value=use_sim_time),
            start_mid360,
            mapping_frame_adapter,
            fastlio_sensor_tf,
            TimerAction(
                period=3.0,
                actions=[fastlio_mapping, fastlio_localization],
            ),
            TimerAction(period=5.0, actions=[start_prior_localizer]),
            start_local_planner,
            motion_gate,
            sdk2_motion_backend,
            start_terrain_analysis,
            start_terrain_analysis_ext,
            start_sensor_scan_generation,
            start_visualization_tools,
            joy_node,
            start_far_navigation,
            start_passive_far_graph,
        ]
    )
