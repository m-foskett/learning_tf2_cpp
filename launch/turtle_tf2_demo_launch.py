from launch import LaunchDescription # Generic launching framework "launch" (not ROS2 specific)
from launch_ros.actions import Node # ROS2 specific launching actions


def generate_launch_description():
    return LaunchDescription([
        # Launching the turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        # Launching the turtle_tf2_broadcaster node
        Node(
            package='learning_tf2_cpp',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
    ])