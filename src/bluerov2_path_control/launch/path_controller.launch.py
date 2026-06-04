from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('namespace', default_value='', description='Optional ROS namespace for the controller node'),
        DeclareLaunchArgument('model_name', default_value='bluerov2'),
        DeclareLaunchArgument('use_bridge', default_value='true', description='Bridge direct thruster topics'),
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([
            FindPackageShare('bluerov2_path_control'), 'config', 'bluerov2_path_control.yaml'
        ])),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('bluerov2_path_control'), 'launch', 'direct_thruster_bridge.launch.py'
        ])),
        launch_arguments={'model_name': LaunchConfiguration('model_name')}.items(),
        condition=IfCondition(LaunchConfiguration('use_bridge')),
    )

    controller = Node(
        package='bluerov2_path_control',
        executable='path_controller',
        name='path_controller',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('config_file'), {'model_name': LaunchConfiguration('model_name')}],
        output='screen',
    )

    return LaunchDescription(args + [bridge, controller])
