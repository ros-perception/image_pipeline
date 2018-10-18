import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    topic = launch.substitutions.LaunchConfiguration('image')
    fps = launch.substitutions.LaunchConfiguration('fps', default='15')
    filename = launch.substitutions.LaunchConfiguration('filename', default= 'output.avi')
    codec = launch.substitutions.LaunchConfiguration('codec', default='MJPG')
    encoding = launch.substitutions.LaunchConfiguration('encoding', default='bgr8')
    return LaunchDescription([
        launch_ros.actions.Node(
            package='image_view',
            node_executable='video_recorder',
            node_name='video_recorder',
            output='screen',
            parameters=[{ 'image': topic}, { 'fps': fps},
                        { 'filename': filename}, { 'codec': codec},
                        { 'encoding': encoding}])
    ])
