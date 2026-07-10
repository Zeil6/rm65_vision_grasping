# Third-party components

This repository focuses on the project-specific integration layer. Large vendor SDKs,
pre-trained weights and generated build artifacts are intentionally not copied here.

| Component | Upstream | Role |
|---|---|---|
| RealMan ROS 2 driver | https://github.com/RealManRobot/ros2_rm_robot | RM65 driver, description and controller |
| yolo_ros | https://github.com/mgonzs13/yolo_ros | 2D/3D object detections |
| ROHand LiteS URDF | https://github.com/oymotion/rohand_lites_urdf_ros2 | Dexterous-hand model |
| Intel RealSense ROS | https://github.com/IntelRealSense/realsense-ros | RGB-D stream and camera frames |

Please follow each upstream project's license and installation instructions. The
calibration values, integration nodes and configuration snapshots in this repository
belong to this project; model meshes and the trained YOLO weight are not redistributed.
