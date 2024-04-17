import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    image_filtering_node = Node(
            package='image_filtering',
            executable='image_filtering_node',
            name='image_filtering_node',
            parameters=[os.path.join(get_package_share_directory('image_filtering'),'params','image_filter_parameters.yaml')],
            output='screen',
        )
    return LaunchDescription([
        image_filtering_node
    ])