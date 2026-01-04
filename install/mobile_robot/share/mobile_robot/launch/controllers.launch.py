import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'mobile_robot'

    # 1. Controller for Robot 1 (Y-Tracker & Cleaner)
    controller1_node = Node(
        package=package_name,
        executable='y_tracker_controller', 
        name='y_tracker_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/cmd_vel', '/robot1/cmd_vel'),
            ('/scan', '/robot1/scan'),
            ('/odom1', '/robot1/odom1'),
        ]
    )

    # 2. Controller for Robot 2 (X-Tracker & Cleaner)
    controller2_node = Node(
        package=package_name,
        executable='x_tracker_controller', 
        name='x_tracker_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/cmd_vel', '/robot2/cmd_vel'),
            ('/scan', '/robot2/scan'),
            ('/odom1', '/robot2/odom1'),
        ]
    )
    
    # Note: Partition Logic nodes are removed as they are now 
    # integrated into the tracker controllers.

    return LaunchDescription([
        controller1_node,
        controller2_node
    ])