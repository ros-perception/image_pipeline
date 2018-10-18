import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    image = launch.substitutions.LaunchConfiguration('image')
    transport = launch.substitutions.LaunchConfiguration('image_transport', default='raw')
    window_name = launch.substitutions.LaunchConfiguration('window_name', default='/camera/image')
    auto_size = launch.substitutions.LaunchConfiguration('auto_size', defalut='false');
    filename_format = launch.substitutions.LaunchConfiguration('filename_format', defalut='frame%04i.jpg')
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'image', description='image topic to view.'),
        launch_ros.actions.Node(
            package='image_view',
            node_executable='image_view',
            node_name='image_view',
            output='screen',
            parameters=[{ 'image': image},
                        { 'image_transport': transport},
                        { 'filename_format': filename_format},
                        { 'window_name': window_name},
                        { 'auto_size': auto_size}]),
    ])
