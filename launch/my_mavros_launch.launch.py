from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node MAVROS
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[
                {'fcu_url': 'udp://:14540@localhost:14557'},  # port do polaczenia z Gazebo
                {'system_id': 1},
                {'component_id': 1},
            ],
        ),
        
        # Node DroneControlNode
        Node(
            package='drone_fly_to_point',
            executable='drone_fly_to_point',
            output='screen',
        ),
    ])
