from launch import LaunchDescription # Generic launching framework "launch" (not ROS2 specific)
from launch.actions import DeclareLaunchArgument # Launch arguments
from launch.substitutions import LaunchConfiguration # Launch configuration
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
        # Declare a launch argument to pass the target frame name
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        # Launching the turtle_tf2_listener node for a second turtle
        Node(
            package='learning_tf2_cpp',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        # Launching the turtle_tf2_listener node to subscribe to the transforms
        Node(
            package='learning_tf2_cpp',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ),
    ])