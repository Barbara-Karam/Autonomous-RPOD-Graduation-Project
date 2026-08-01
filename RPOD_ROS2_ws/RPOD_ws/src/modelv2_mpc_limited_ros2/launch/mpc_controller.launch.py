from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='modelv2_mpc_ros2',
            executable='mpc_controller_node',
            name='modelv2_mpc_controller',
            output='screen',
            remappings=[
                ('cv/relative_pose', '/cv/relative_pose'),
                ('motor/angular_velocities', '/motor/angular_velocities'),
            ],
        ),
    ])
