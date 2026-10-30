from launch import LaunchDescription
from launch_ros.actions import Node

ISU_ID = '439129'


def generate_launch_description():
    return LaunchDescription([
        # --- Пространство имён 1
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='ns1_' + ISU_ID,
            name='sim',
            output='screen'
        ),
        Node(
            package='mybestcontroller',
            executable='lab1_controller',
            namespace='ns1_' + ISU_ID,
            name='lab1_controller',
            output='screen'
        ),
        # --- Пространство имён 2
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='ns2_' + ISU_ID,
            name='sim',
            output='screen'
        ),
        Node(
            package='mybestcontroller',
            executable='lab1_controller',
            namespace='ns2_' + ISU_ID,
            name='lab1_controller',
            output='screen'
        ),
    ])