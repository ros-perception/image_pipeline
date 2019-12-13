from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # TODO(jacobperron): Include image_proc launch file when it exists
    return LaunchDescription([
        DeclareLaunchArgument(
            name='approximate_sync', default_value='False',
            description='Whether to use approximate synchronization of topics. Set to true if '
                'the left and right cameras do not produce exactly synced timestamps.'
        ),
        ComposableNodeContainer(
            package='rclcpp_components', node_executable='component_container',
            node_name='stereo_image_proc_container', node_namespace='',
            composable_node_descriptions=[
                ComposableNode(
                    package='stereo_image_proc',
                    node_plugin='stereo_image_proc::DisparityNode',
                    parameters=[{'approximate_sync': LaunchConfiguration('approximate_sync')}]
                ),
                ComposableNode(
                    package='stereo_image_proc',
                    node_plugin='stereo_image_proc::PointCloudNode',
                    parameters=[{'approximate_sync': LaunchConfiguration('approximate_sync')}]
                ),
            ],
        ),
    ])
