from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    # Specify the package name
    pkg_name = 'armatronix'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    # Path to the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'armatronix_gazebo.urdf')

    with open(urdf_file, 'r') as file:
        urdf_content = file.read()

    # Path to the YAML config file for the controller
    controller_config = os.path.join(pkg_share, 'config', 'diff_drive_controller.yaml')
    arm_trajectory_config = os.path.join(pkg_share, 'config', 'joint_controller.yaml')

    # Launch Gazebo with the required ROS2 plugins
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros').find('gazebo_ros'), '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'extra_gazebo_args': '-slibgazebo_ros_factory.so'}.items()
    )

    # Launch the robot_state_publisher to publish joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content}]
        # arguments=[urdf_content]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    # Spawn the robot using the URDF file
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'armatronix_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    # Load the controller by passing controller parameters via the parameter server
   # Load the controller by spawning the diff_drive_controller
    diffdrive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
        remappings=[
            ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'),  # Remap cmd_vel_unstamped to cmd_vel
            ('/diff_drive_controller/odom', '/odom')  # Remap odom to standard /odom
        ]
    )

    # Add the controller_manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[urdf_file, controller_config],
        output='screen'
    )
    
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
        spawn_entity,
        controller_manager,
        diffdrive_spawner,
        joint_trajectory_spawner
    ])