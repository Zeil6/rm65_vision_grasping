#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
vision_tf_node.py - 眼在手上(Eye-in-Hand)机械臂抓取项目的视觉TF转换节点
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from yolo_msgs.msg import DetectionArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_geometry_msgs import do_transform_point


class VisionTFNode(Node):
    def __init__(self):
        super().__init__('vision_tf_node')

        self.declare_parameter('source_frame', 'camera_link')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('input_topic', '/yolo/detections_3d')
        self.declare_parameter('output_topic', '/grasp_target_pose')

        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.get_logger().info(f"源坐标系: {self.source_frame}")
        self.get_logger().info(f"目标坐标系: {self.target_frame}")
        self.get_logger().info(f"输入话题: {self.input_topic}")
        self.get_logger().info(f"输出话题: {self.output_topic}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # YOLO 检测输入 QoS
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 关键修复：
        # /grasp_target_pose 与 grasp_control_node 统一使用 VOLATILE
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subscription = self.create_subscription(
            DetectionArray,
            self.input_topic,
            self.detection_callback,
            detection_qos
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            self.output_topic,
            pose_qos
        )

        self.get_logger().info("视觉TF节点已启动，等待YOLO检测结果...")

    def detection_callback(self, msg: DetectionArray):
        if len(msg.detections) == 0:
            self.get_logger().debug("未检测到目标，等待下一帧...")
            return

        try:
            detection = msg.detections[0]

            object_x = detection.bbox3d.center.position.x
            object_y = detection.bbox3d.center.position.y
            object_z = detection.bbox3d.center.position.z

            self.get_logger().info(
                f"检测到目标在{self.source_frame}下的坐标: "
                f"({object_x:.4f}, {object_y:.4f}, {object_z:.4f})"
            )

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.source_frame,
                    rclpy.time.Time()
                )
            except TransformException as e:
                self.get_logger().warning(
                    f"无法获取TF变换 ({self.source_frame} -> {self.target_frame}): {str(e)}"
                )
                self.get_logger().warning("请检查TF树是否正确发布")
                return

            source_point = PointStamped()
            source_point.header.stamp = self.get_clock().now().to_msg()
            source_point.header.frame_id = self.source_frame
            source_point.point.x = object_x
            source_point.point.y = object_y
            source_point.point.z = object_z

            transformed_point = do_transform_point(source_point, transform)

            self.get_logger().info(
                f"转换后目标在{self.target_frame}下的坐标: "
                f"({transformed_point.point.x:.4f}, "
                f"{transformed_point.point.y:.4f}, "
                f"{transformed_point.point.z:.4f})"
            )

            grasp_pose = PoseStamped()
            grasp_pose.header.stamp = self.get_clock().now().to_msg()
            grasp_pose.header.frame_id = self.target_frame

            grasp_pose.pose.position.x = transformed_point.point.x
            grasp_pose.pose.position.y = transformed_point.point.y
            grasp_pose.pose.position.z = transformed_point.point.z

            grasp_pose.pose.orientation.w = 1.0
            grasp_pose.pose.orientation.x = 0.0
            grasp_pose.pose.orientation.y = 0.0
            grasp_pose.pose.orientation.z = 0.0

            self.pose_publisher.publish(grasp_pose)

            self.get_logger().info(
                f"已发布抓取目标位姿到话题: {self.output_topic}"
            )

        except IndexError as e:
            self.get_logger().warning(f"访问检测列表时发生索引错误: {str(e)}")
            return

        except AttributeError as e:
            self.get_logger().error(f"消息结构错误，请检查消息类型: {str(e)}")
            return

        except Exception as e:
            self.get_logger().error(f"处理检测结果时发生未知错误: {str(e)}")
            return


def main(args=None):
    rclpy.init(args=args)
    vision_tf_node = VisionTFNode()

    try:
        rclpy.spin(vision_tf_node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            vision_tf_node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()