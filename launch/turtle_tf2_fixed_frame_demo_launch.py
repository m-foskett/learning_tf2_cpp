import os

from ament_index_python.packages import get_package_share_directory # Get the package share directory

from launch import LaunchDescription # Generic launching framework "launch" (not ROS2 specific)
from launch.actions import IncludeLaunchDescription # Include other launch files
from launch.launch_description_sources import PythonLaunchDescriptionSource # Launch description source

from launch_ros.actions import Node # ROS2 specific launching actions


def generate_launch_description():
    # Store nodes created by the included launch file
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('learning_tf2_cpp'), 'launch'),
            '/turtle_tf2_demo_launch.py']),
            launch_arguments={'target_frame': 'carrot1'}.items(),
        )

    return LaunchDescription([
        # Include the nodes from the turtle_tf2_demo_launch.py file
        demo_nodes,
        # Launching the fixed_frame_tf2_broadcaster node
        Node(
            package='learning_tf2_cpp',
            executable='fixed_frame_tf2_broadcaster',
            name='fixed_broadcaster',
        ),
    ])