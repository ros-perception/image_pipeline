import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    stereo = launch.substitutions.LaunchConfiguration('stereo')
    transport = launch.substitutions.LaunchConfiguration('image_transport', default='raw')
    image = launch.substitutions.LaunchConfiguration('image', default='image')
    filename_format = launch.substitutions.LaunchConfiguration('filename_format', default= "%s%04i.jpg")
    queue_size  = launch.substitutions.LaunchConfiguration('queue_size', default=5)
    approximate_sync = launch.substitutions.LaunchConfiguration('approximate_sync', default='false')
    disparity = launch.substitutions.LaunchConfiguration("disparity", default="/disparity")
    return LaunchDescription([
        launch_ros.actions.Node(
            package='image_view',
            node_executable='stereo_view',
            node_name='stereo_view',
            output='screen',
            parameters=[{ 'image': image}, { 'image_transport': transport},
                        { 'stereo': stereo}, { 'queue_size': queue_size},
                        { 'approximate_sync': approximate_sync},
                        { 'filename_format': filename_format},
                        { 'disparity': disparity}]),
    ])
