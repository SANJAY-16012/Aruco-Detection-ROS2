# ArUco Docking Node (ROS 2 Humble)

This package provides a **ROS 2 node for docking and localization using ArUco markers** with an **Intel RealSense camera**.  
It detects a specified ArUco marker, estimates its 3D pose, publishes the marker’s pose in the `map` frame, and visualizes it in RViz.

---

## 🔹 Features
- Detects **ArUco markers** in the camera feed
- Estimates marker **pose (3D position + orientation)** using camera intrinsics
- Publishes:
  - `PoseStamped` → `/aruco_marker_pose`
  - `Float32` → `/aruco_marker_distance`
  - `Point` → `/aruco_marker_center`
  - `Marker` → `/obstacle_visual` (for RViz visualization)
- Uses **TF2** to transform poses from `camera_link` to `map` frame
- Visualizes detection results in RViz and OpenCV window

---

## 🔹 Subscribed Topics
- **Intel RealSense streams** (depth + color) via `pyrealsense2`

---

## 🔹 Published Topics
| Topic                  | Type                        | Description                                   |
|-------------------------|-----------------------------|-----------------------------------------------|
| `/aruco_marker_pose`    | `geometry_msgs/PoseStamped` | Pose of the detected marker (in map frame)    |
| `/aruco_marker_distance`| `std_msgs/Float32`          | Distance to the marker (from depth camera)    |
| `/aruco_marker_center`  | `geometry_msgs/Point`       | Pixel coordinates of marker center            |
| `/obstacle_visual`      | `visualization_msgs/Marker` | RViz marker (red sphere at marker location)   |

---

## 🔹 Requirements
- ROS 2 Humble
- OpenCV with ArUco (`opencv-contrib-python`)
- Intel RealSense SDK (`pyrealsense2`)
- `tf2_ros` and `tf2_geometry_msgs`

---
