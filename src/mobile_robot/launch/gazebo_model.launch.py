import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess # <--- Added ExecuteProcess

def generate_launch_description():
    namePackage = 'mobile_robot'
    
    model_dir = os.path.join(get_package_share_directory(namePackage), 'model')
    world_path = os.path.join(get_package_share_directory(namePackage), 'worlds', 'arena_walls.world')
    
    robot1_xacro_path = os.path.join(model_dir, 'robot1.xacro')
    robot2_xacro_path = os.path.join(model_dir, 'robot2.xacro')

    def process_xacro_with_name(robot_name, xacro_file_path):
        command = ['xacro', xacro_file_path, f'robot_name:={robot_name}']
        try:
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            return result.stdout
        except subprocess.CalledProcessError as e:
            print(f"XACRO error: {e.stderr}")
            raise e
    
    robot1_description = process_xacro_with_name('robot1', robot1_xacro_path)
    robot2_description = process_xacro_with_name('robot2', robot2_xacro_path)

    gazeboLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v4 {world_path}', 'on_exit_shutdown': 'true'}.items()
    )

    # === ROBOT SPAWNERS ===
    spawnModelNodeGazebo1 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'robot1', '-topic', 'robot1/robot_description', '-x', '-9.4', '-y', '9.4', '-z', '0.05', '-Y', '0.0'],
        output='screen',
    )
    
    spawnModelNodeGazebo2 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'robot2', '-topic', 'robot2/robot_description', '-x', '-9.4', '-y', '8.4', '-z', '0.05', '-Y', '-1.57'],
        output='screen',
    )

    nodeRobotStatePublisher1 = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='robot1', output='screen',
        parameters=[{'robot_description': robot1_description, 'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    nodeRobotStatePublisher2 = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='robot2', output='screen',
        parameters=[{'robot_description': robot2_description, 'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )
    
    bridge_params = os.path.join(get_package_share_directory(namePackage), 'parameters', 'bridge_parameters.yaml')
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    chatNode1 = Node(package=namePackage, executable='robot_chat', namespace='robot1', output='screen')
    chatNode2 = Node(package=namePackage, executable='robot_chat', namespace='robot2', output='screen')
    # === ROBOT TRACKER NODES ===
    # These start your y_axis_tracker script for each robot
    trackerNode1 = Node(
        package=namePackage, 
        executable='y_tracker', 
        namespace='robot1', 
        output='screen'
    )

    trackerNode2 = Node(
        package=namePackage, 
        executable='x_tracker', 
        namespace='robot2', 
        output='screen'
    )

    # === NEW: SPAWN WASTE SCRIPT ===
    # This runs "python3 <path_to_spawn_waste.py>"
    spawn_waste_cmd = ExecuteProcess(
        cmd=['python3', os.path.join(get_package_share_directory(namePackage), 'launch', 'spawn_waste.py')],
        output='screen'
    )

    # We wrap it in a TimerAction to wait 5 seconds for Gazebo to load before spawning waste
    delayed_waste_spawn = TimerAction(
        period=5.0,
        actions=[spawn_waste_cmd]
    )

    return LaunchDescription([
        gazeboLaunch,
        nodeRobotStatePublisher1, spawnModelNodeGazebo1, chatNode1,
        nodeRobotStatePublisher2, spawnModelNodeGazebo2, chatNode2,
        start_gazebo_ros_bridge_cmd,
        delayed_waste_spawn 
    
    ])