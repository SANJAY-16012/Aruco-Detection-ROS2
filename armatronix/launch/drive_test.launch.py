from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Specify the package name
    pkg_name = 'armatronix'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'armatronix_gazebo.urdf')

    with open(urdf_file, 'r') as file:
        urdf_content = file.read()

    arm_trajectory_config = os.path.join(pkg_share, 'config', 'joint_controller.yaml')
    gz_bridge_config = os.path.join(get_package_share_directory(pkg_name),'config','gz_bridge.yaml')
    world = "empty.sdf"
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world}'}.items()
    )

    # Launch the robot_state_publisher to publish joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content}]
    )

    # Spawner for joint_state_broadcaster (to publish joint states)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawn the robot using the URDF file
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-entity', 'armatronix_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={gz_bridge_config}',
        ]
    )

    # Add the controller_manager node for the arm manipulation
    controller_manager = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[urdf_file, arm_trajectory_config],
    output='screen',
    )

    
    # Spawner for the joint_trajectory_controller (arm manipulation controller)
    joint_trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen',
        parameters=[arm_trajectory_config]
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        controller_manager,
        spawn_entity,
        ros_gz_bridge,
        joint_trajectory_spawner 
    ])
