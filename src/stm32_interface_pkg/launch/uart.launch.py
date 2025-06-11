from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stm32_interface_pkg',
            executable='stm32_uart_node',
            name='stm32_uart_node',
            output='screen',
            parameters=[]  # You can pass port/baudrate as params here later
        )
    ])
