import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    main_launch = LaunchDescription()
    
    pcl_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pcl_detector'), 'launch', 'pcl_detector_launch.py')
        ),
    )

    target_tracking_node = Node(
            package='target_tracking',
            executable='target_tracking_node',
            name='target_tracking_node',
            parameters=[os.path.join(get_package_share_directory('target_tracking'),'params','target_tracking_params.yaml')],
            output='screen',
        )
    target_tracking_visualization_node = Node(
            package='target_tracking_visualization',
            executable='target_tracking_visualization_node',
            name='target_tracking_visualization_node',
            output='screen',
        )
    landmark_server_node = Node(
            package='landmark_server',
            executable='landmark_server_node',
            name='landmark_server_node',
            output='screen',
        )
    
    main_launch.add_action(pcl_detector_launch)
    
    main_launch.add_action(target_tracking_node)
    main_launch.add_action(target_tracking_visualization_node)
    main_launch.add_action(landmark_server_node)
    return main_launch