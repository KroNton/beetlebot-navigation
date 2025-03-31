from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

# Paths to configuration files
nav2_yaml = os.path.join(get_package_share_directory('beetlebot_navigation'), 'config', 'planner_server.yaml')
controller_yaml = os.path.join(get_package_share_directory('beetlebot_navigation'), 'config', 'controller.yaml')
bt_navigator_yaml = os.path.join(get_package_share_directory('beetlebot_navigation'), 'config', 'bt_navigator.yaml')
recovery_yaml = os.path.join(get_package_share_directory('beetlebot_navigation'), 'config', 'recovery.yaml')
def generate_launch_description():
    # Define the RViz node
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_yaml])

    # Define the map server node
    controller_node = Node(
        name='controller_server',
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[controller_yaml])
    
    bt_nav_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml])
    
    recovery_nav_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        parameters=[recovery_yaml],
        output='screen'),

    # Define the lifecycle manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['planner_server', 'controller_server', 'bt_navigator', 'behavior_server']}],
    )

    # Delay the launch of these nodes by 5 seconds
    delayed_nodes = TimerAction(
        period=2.0,  # Delay duration in seconds
        actions=[
            planner_node,
            controller_node,
            bt_nav_node,
            recovery_nav_node,
            lifecycle_manager_node
        ]
    )

    # Add everything to the LaunchDescription
    return LaunchDescription([
        delayed_nodes   # Launch other nodes after a 1-second delay
    ])