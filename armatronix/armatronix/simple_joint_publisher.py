import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Publisher to the joint trajectory controller
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory)
        
        # Define the joint names in the correct order
        self.joint_names = ['joint_1', 'joint_2', 'joint_4', 'gripper_joint']
        self.get_logger().info('TrajectoryPublisher node has been started.')

    def publish_trajectory(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Define trajectory points
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.5, 0.5, 0.1]
        point1.time_from_start.sec = 2  # Move to this point in 2 seconds

        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 1.0, 0.2, 0.3]
        point2.time_from_start.sec = 4  # Move to this point in 4 seconds

        point3 = JointTrajectoryPoint()
        point3.positions = [1.0, 0.5, 0.0, -0.5]
        point3.time_from_start.sec = 6  # Move to this point in 6 seconds

        point4 = JointTrajectoryPoint()
        point4.positions = [-1.0, 1.5, -0.8, -0.8]
        point4.time_from_start.sec = 10  # Move to this point in 6 seconds

        point5 = JointTrajectoryPoint()
        point5.positions = [1.4, -1.5, -0.3, -0.6]
        point5.time_from_start.sec = 12  # Move to this point in 6 seconds

        # Add points to the trajectory
        trajectory_msg.points = [point1, point2, point3, point4, point5]

        # Publish the trajectory
        self.publisher_.publish(trajectory_msg)
        self.get_logger().info('Published trajectory.')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
