import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    topic = launch.substitutions.LaunchConfiguration('image')
    filename_format = launch.substitutions.LaunchConfiguration('filename_format', default= "frame")
    encoding  = launch.substitutions.LaunchConfiguration('encoding', default='bgr8')
    save_all_image = launch.substitutions.LaunchConfiguration('save_all_image', default='true')
    return LaunchDescription([
        launch_ros.actions.Node(
            package='image_view',
            node_executable='image_saver',
            node_name='image_saver',
            output='screen',
            parameters=[{ 'image': topic}, { 'encoding': encoding},
                        { 'filename_format': filename_format},
                        { 'save_all_image': save_all_image}])
    ])
