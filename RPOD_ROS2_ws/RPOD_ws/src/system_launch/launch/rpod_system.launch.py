from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    static_mpc_node = Node(
        package='modelv2_mpc_limited_ros2',
        executable='mpc_controller_node',
        name='static_mpc_controller',
        output='screen',
        remappings=[
            ('wheel_angular_velocities',
             '/mpc/static/wheel_angular_velocities')
        ]
    )

    tumbling_mpc_node = Node(
        package='statespace_chaser_mpc_ros2',
        executable='mpc_controller_node',
        name='tumbling_mpc_controller',
        output='screen',
        remappings=[
            ('wheel_angular_velocities',
             '/mpc/tumbling/wheel_angular_velocities')
        ]
    )

    mpc_supervisor_node = Node(
        package='mpc_super_visor',
        executable='mpc_super_visor_node',
        name='mpc_supervisor_node',
        output='screen',
        parameters=[{
            'active_mpc': 'static',
            'switch_zero_cycles': 5
        }]
    )

    speed_processing_node = Node(
        package='speed_processing',
        executable='speed_processing_node',
        name='speed_processing_node',
        output='screen'
    )

    motor_uart_node = Node(
        package='motor_uart_package',
        executable='motor_uart_node',
        name='motor_uart_node',
        output='screen',
        parameters=[{
            'uart_port': '/dev/ttyUSB0',
            'baud_rate': 921600
        }]
    )

    return LaunchDescription([
        static_mpc_node,
        tumbling_mpc_node,
        mpc_supervisor_node,
        speed_processing_node,
        motor_uart_node
    ])