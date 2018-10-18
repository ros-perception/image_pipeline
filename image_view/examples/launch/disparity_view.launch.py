import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    image = launch.substitutions.LaunchConfiguration('image')
    auto_size = launch.substitutions.LaunchConfiguration('auto_size', default='false');
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'image', description='Disparity image topic to view.'),
        launch_ros.actions.Node(
            package='image_view',
            node_executable='disparity_view',
            node_name='disparity_view',
            output='screen',
            parameters=[{ 'image': image},
                        { 'window_name': image},
                        { 'auto_size': auto_size}])
    ])