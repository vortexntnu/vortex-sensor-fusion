import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    target_tracking_node = Node(
        package='target_tracking',
        executable='target_tracking_node',
        name='target_tracking_node',
        parameters=[os.path.join(get_package_share_directory('target_tracking'), 'params', 'target_tracking_params.yaml')],
        output='screen',
    )
    
    enable_visualization = LaunchConfiguration('enable_visualization')
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='False',
        description='enable visualization'
    )

    target_tracking_visualization_node = Node(
        package='target_tracking_visualization',
        executable='target_tracking_visualization_node',
        name='target_tracking_visualization_node',
        output='screen',
        condition=IfCondition(enable_visualization),
    )
        
    return LaunchDescription([
        enable_visualization_arg,
        target_tracking_node,
        target_tracking_visualization_node,
    ])