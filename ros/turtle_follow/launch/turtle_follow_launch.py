from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen'
        ),
        
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='turtle_teleop_key',
            output='screen',
            prefix='xterm -e'  
        ),
        
        Node(
            package='turtle_follow',
            executable='turtle_follow_node',
            name='turtle_follow_node',
            output='screen'
        )
    ])