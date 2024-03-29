import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pcl_detector_node = Node(
            package='pcl_detector',
            executable='pcl_detector_node',
            name='pcl_detector_node',
            parameters=[os.path.join(get_package_share_directory('pcl_detector'),'params','pcl_detector_params.yaml')],
            output='screen',
        )
    return LaunchDescription([
        pcl_detector_node
    ])