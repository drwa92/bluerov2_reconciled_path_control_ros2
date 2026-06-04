from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model_name').perform(context)
    bridge_args = []
    for i in range(1, 7):
        bridge_args.append(
            f'/model/{model}/joint/thruster{i}_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double'
        )
    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=bridge_args,
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_name', default_value='bluerov2'),
        OpaqueFunction(function=launch_setup),
    ])
