import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    target_tracking_node = Node(
        package='target_tracking',
        executable='target_tracking_node',
        name='target_tracking_node',
        parameters=[os.path.join(get_package_share_directory('sensor_fusion_launch'),'params','target_tracking_params.yaml')],
        output='screen',
    )
    
    enable_pcl_detector = LaunchConfiguration('pcl_detector')
    enable_pcl_detector_arg = DeclareLaunchArgument(
        'pcl_detector',
        default_value='true',
        description='Enable PCL detector'
    )

    enable_landmark_server = LaunchConfiguration('landmark_server')
    enable_landmark_server_arg = DeclareLaunchArgument(
        'landmark_server',
        default_value='false',
        description='Enable landmark server',
    )
    
    enable_visualization = LaunchConfiguration('visualization')
    enable_visualization_arg = DeclareLaunchArgument(
        'visualization',
        default_value='false',
        description='Enable visualization'
    )
    
    pcl_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pcl_detector'), 'launch', 'pcl_detector_launch.py')
        ),
        condition=IfCondition(enable_pcl_detector)
    )
        
    landmark_server_node = Node(
        package='landmark_server',
        executable='landmark_server_node',
        name='landmark_server_node',
        output='screen',
        condition=IfCondition(enable_landmark_server),
    )
        
    target_tracking_visualization_node = Node(
        package='target_tracking_visualization',
        executable='target_tracking_visualization_node',
        name='target_tracking_visualization_node',
        output='screen',
        condition=IfCondition(enable_visualization),
    )
        
    return LaunchDescription([
        enable_pcl_detector_arg,
        enable_landmark_server_arg,
        enable_visualization_arg,
        target_tracking_node,
        pcl_detector_launch,
        landmark_server_node,
        target_tracking_visualization_node,
    ])