ArUco Docking ROS2 Node

This ROS2 node, aruco_docking_node, is designed to interface with a RealSense camera and detect ArUco markers for robotics applications such as docking and obstacle detection. It performs the following tasks:

    ArUco Marker Detection: Detects ArUco markers using the RealSense RGB camera.
    Pose Estimation: Calculates the position of detected markers relative to the camera frame and transforms it into the map frame.
    Obstacle Visualization: Publishes the detected marker's position as a Marker to visualize obstacles in the environment.
    Distance Calculation: Measures and publishes the distance to the detected marker.
    Marker Center Tracking: Tracks and publishes the center coordinates of detected markers.

Dependencies

The following packages are required to run this node:

    ROS 2 Humble or newer
    OpenCV (for ArUco marker detection)
    Pyrealsense2 (for interfacing with the Intel RealSense camera)
    tf2_geometry_msgs (for coordinate transformations)
    cv2 (for marker detection and visualization)
    geometry_msgs, std_msgs, visualization_msgs (for ROS2 message types)

To install the necessary dependencies, you can run the following commands:

pip install opencv-python pyrealsense2

Ensure ROS2 and necessary packages (like geometry_msgs, std_msgs, etc.) are set up.
Setup

    RealSense Camera Configuration:
        This node uses an Intel RealSense camera for capturing both depth and color frames. Make sure your RealSense camera is connected and recognized by your system.

    Camera Calibration:
        The camera matrix (cameraMatrix) and distortion coefficients (distCoeffs) are preconfigured in the node. These values are based on the camera's calibration and may need to be adjusted if you're using a different camera model or calibration.

    ArUco Marker Setup:
        The node detects ArUco markers of ID 22 by default. You can change this marker ID in the code as needed.

    TF Setup:
        The node assumes that the camera_link and map frames exist in the TF tree. Ensure that these frames are being broadcast by your robot's TF setup.

Topics

The node publishes to the following topics:

    /aruco_marker_pose (PoseStamped): The pose (position and orientation) of the detected ArUco marker in the map frame.
    /aruco_marker_distance (Float32): The distance to the detected ArUco marker.
    /aruco_marker_center (Point): The 2D image coordinates (pixel location) of the center of the detected marker.
    /obstacle_visual (Marker): Visualization of the detected ArUco marker as a sphere in the map frame.

Launch Instructions

    Run the ROS2 Node: After setting up the ROS2 workspace, build your package and run the node using:

    ros2 run your_package_name aruco_docking_node

    Visualize the Marker: To visualize the detected ArUco markers, you can use rviz2 to subscribe to the /obstacle_visual topic. Make sure to set the fixed frame to map and add a Marker display.

    Visualize the Pose: You can also visualize the detected marker poses by subscribing to the /aruco_marker_pose topic in rviz2.

    View Real-Time Video Feed: The node also opens an OpenCV window displaying the real-time video feed with detected ArUco markers. Press q to close the window.

Example Workflow

    Connect the Intel RealSense camera to your computer.

    Run the node:

    ros2 run your_package_name aruco_docking_node

    The node will start processing frames from the RealSense camera, detect ArUco markers, and publish the pose, distance, and marker center data to the respective topics.

    Open rviz2 to visualize the detected marker's pose and its obstacle representation.

Troubleshooting

    No Marker Detection: Ensure that the camera is facing the ArUco marker and that the marker is within the camera's field of view.
    TF Transform Error: Make sure the camera_link and map frames are correctly published in your robot's TF tree.

Customization

    Marker ID: You can modify the marker ID in the code if you are using a different ArUco marker.
    Marker Size: If you are using markers of a different size, update the marker_size parameter in the pose estimation method (my_estimatePoseSingleMarkers).
    Camera Calibration: If using a different camera or configuration, you may need to adjust the cameraMatrix and distCoeffs to match your specific calibration.
