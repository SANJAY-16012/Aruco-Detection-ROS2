import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import tf2_geometry_msgs
import pyrealsense2 as rs
from cv2 import aruco
from geometry_msgs.msg import PoseStamped, Point, Pose
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener, LookupException
from visualization_msgs.msg import Marker

class ArucoDockingNode(Node):

    def __init__(self):
        super().__init__('aruco_docking_node')

        # ROS2 Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_marker_pose', 10)
        self.distance_pub = self.create_publisher(Float32, '/aruco_marker_distance', 10)
        self.marker_center_pub = self.create_publisher(Point, '/aruco_marker_center', 10)
        self.marker_obstacle_pub = self.create_publisher(Marker, '/obstacle_visual', 10)

        self.cameraMatrix = np.array([[398.47500394,   0,         336.99146777],
                                    [  0,         397.49105484, 249.26603721],
                                    [  0,           0,           1        ]])
        self.distCoeffs = np.array([[-0.04080487, 0.08105768, -0.00041492, 0.00470619, -0.03644504]])

        # RealSense Setup
        self.width, self.height = 640, 360
        self.pipeline = rs.pipeline()
        self.realsense_cfg = rs.config()
        self.realsense_cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, 30)
        self.realsense_cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.realsense_cfg)

        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        # ArUco Detection Setup
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def my_estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        rvecs = []
        tvecs = []
        trash = []

        for c in corners:
            _, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
        return rvecs, tvecs, trash


    def marker_filled(self, points: Point):

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "basic_shapes"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = points.x, points.y, points.z
       
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # marker.lifetime = rclpy.duration.Duration(seconds=1.0)

        return marker


    def tf_transform(self, tvec):
        pose_msg = PoseStamped()
        p = Pose()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_link"
        p.position.x = float(tvec[0][0][0])
        p.position.y = float(tvec[0][1][0])
        p.position.z = float(tvec[0][2][0])
        p.orientation.y = 0.0
        p.orientation.x = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
        pose_msg.pose = p
        try:
            transform = self.tf_buffer.lookup_transform('map', 'camera_link', rclpy.time.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(pose_msg, transform)
            marker = self.marker_filled(transformed_pose.pose.position)

            self.marker_obstacle_pub.publish(marker)
            self.pose_pub.publish(transformed_pose)
        except LookupException as e:
            self.get_logger().warn(f"Could not transform pose: {str(e)}")

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        img_rgb = np.asanyarray(color_frame.get_data())
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(img_gray)
        if ids is not None:
            for i, corner in enumerate(corners):
                if ids[i][0] == 22:  # Adjust according to your marker ID
                    rvec, tvec, _ = self.my_estimatePoseSingleMarkers(corner, 0.05, self.cameraMatrix, self.distCoeffs)

                    self.tf_transform(tvec)


                    # Publish distance to marker
                    cx = float(corner[0][0][0])
                    cy = float(corner[0][0][1])
                    dist = depth_frame.get_distance(int(cx), int(cy))  # Convert to mm
                    self.distance_pub.publish(Float32(data=dist))

                    # Publish center of the marker in the image
                    center_point = Point(x=cx, y=cy)
                    self.marker_center_pub.publish(center_point)

                    # Optionally draw and display the markers for debugging
                    cv2.aruco.drawDetectedMarkers(img_rgb, [corner], ids[i])

        cv2.imshow("ArUco Detection", img_rgb)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    aruco_docking_node = ArucoDockingNode()
    rclpy.spin(aruco_docking_node)

    aruco_docking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
