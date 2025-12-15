import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

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

    # === FIXED SPAWN POSITIONS ===
    
    # Robot 1 (Y-Tracker): Spawn in OPEN SPACE
    spawnModelNodeGazebo1 = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-name', 'robot1',
            '-topic', 'robot1/robot_description',
            '-x', '-9.4',   # Changed from -9.4
            '-y', '9.4',   # Changed from 9.4
            '-z', '0.2',   # Drop from height to prevent stuck wheels
            '-Y', '0.0' # Face +Y
        ],
        output='screen',
    )
    
    # Robot 2 (X-Tracker): Spawn facing X AXIS
    spawnModelNodeGazebo2 = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-name', 'robot2',
            '-topic', 'robot2/robot_description',
            '-x', '-9.4',  # Start back a bit
            '-y', '8.4',  # Away from Robot 1
            '-z', '0.2',
            '-Y', '-1.57'  # Face +X (So X-Tracker works)
        ],
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

    return LaunchDescription([
        gazeboLaunch,
        nodeRobotStatePublisher1, spawnModelNodeGazebo1,
        nodeRobotStatePublisher2, spawnModelNodeGazebo2,
        start_gazebo_ros_bridge_cmd
    ])