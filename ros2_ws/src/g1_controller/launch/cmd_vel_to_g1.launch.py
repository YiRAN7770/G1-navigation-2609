from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

def generate_launch_description():
    # Declare launch arguments
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='eno1',
        description='Network interface for G1 robot communication'
    )
    
    g1_loco_client_path_arg = DeclareLaunchArgument(
        'g1_loco_client_path',
        default_value=EnvironmentVariable('G1_LOCO_CLIENT_BIN', default_value=''),
        description='Path to g1_loco_client executable'
    )

    unitree_sdk_lib_path_arg = DeclareLaunchArgument(
        'unitree_sdk_lib_path',
        default_value=EnvironmentVariable('UNITREE_SDK_LIB_DIR', default_value=''),
        description='Path to Unitree SDK shared libraries'
    )
    
    command_timeout_arg = DeclareLaunchArgument(
        'command_timeout',
        default_value='0.1',
        description='Minimum time between commands in seconds (safety debouncing)'
    )

    enable_continuous_gait_arg = DeclareLaunchArgument(
        'enable_continuous_gait',
        default_value='false',
        description='Explicitly arm G1 continuous gait on each nonzero command'
    )
    
    max_x_linear_velocity_arg = DeclareLaunchArgument(
        'max_x_linear_velocity',
        default_value='0.5',
        description='Maximum linear velocity (x) in m/s (safety limit)'
    )

    max_y_linear_velocity_arg = DeclareLaunchArgument(
        'max_y_linear_velocity',
        default_value='0.15',
        description='Maximum linear velocity (y) in m/s (safety limit)'
    )
    
    max_angular_velocity_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='0.5',
        description='Maximum angular velocity (z) in rad/s (safety limit)'
    )
    
    # Create the node
    cmd_vel_to_g1_node = Node(
        package='g1_controller',
        executable='cmd_vel_to_g1',
        name='cmd_vel_to_g1',
        output='screen',
        parameters=[{
            'network_interface': LaunchConfiguration('network_interface'),
            'g1_loco_client_path': LaunchConfiguration('g1_loco_client_path'),
            'unitree_sdk_lib_path': LaunchConfiguration('unitree_sdk_lib_path'),
            'command_timeout': LaunchConfiguration('command_timeout'),
            'enable_continuous_gait': LaunchConfiguration('enable_continuous_gait'),
            'max_x_linear_velocity': LaunchConfiguration('max_x_linear_velocity'),
            'max_y_linear_velocity': LaunchConfiguration('max_y_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
        }]
    )
    
    return LaunchDescription([
        network_interface_arg,
        g1_loco_client_path_arg,
        unitree_sdk_lib_path_arg,
        command_timeout_arg,
        enable_continuous_gait_arg,
        max_x_linear_velocity_arg,
        max_y_linear_velocity_arg,
        max_angular_velocity_arg,
        cmd_vel_to_g1_node
    ])
