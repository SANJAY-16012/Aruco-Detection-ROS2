from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the package and the URDF file
    pkg_description = get_package_share_directory('armatronix')
    urdf_file = os.path.join(pkg_description, "urdf", 'armatronix_gazebo.urdf')
    rviz_config_path = "/home/kira/sample_ws/src/armatronix/show.rviz"

    # Read the URDF file
    with open(urdf_file, 'r') as file:
        urdf_content = file.read()

    # Define the Joint State Publisher Node
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="JSP",
        output="screen",
    )

    # Define the Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="RSP",
        output="screen",
        parameters=[{'robot_description': urdf_content}]
    )

    # Define the RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_gui",
        output="screen",
        arguments=['-d',rviz_config_path]
    )

    # Return the LaunchDescription containing all nodes
    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])

if __name__ == '__main__':
    generate_launch_description()
