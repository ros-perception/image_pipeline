import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    topic = launch.substitutions.LaunchConfiguration('image')
    filename_format = launch.substitutions.LaunchConfiguration('format_string', default= 'frame%04i.jpg')
    sec_per_frame = launch.substitutions.LaunchConfiguration('sec_per_frame', default='0.1')
    return LaunchDescription([
        launch_ros.actions.Node(
            package='image_view',
            node_executable='extract_images',
            node_name='extract_images',
            output='screen',
            parameters=[{ 'image': topic},
                        { 'sec_per_frame': sec_per_frame},
                        { 'filename_format': filename_format}])
    ])
