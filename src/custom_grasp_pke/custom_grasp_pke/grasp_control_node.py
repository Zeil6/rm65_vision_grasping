#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
grasp_control_node.py - 机械臂抓取控制节点（完整可替换版）

修复目标：
1. 不在 __init__ 中阻塞等待 Action / Service，避免节点初始化卡死
2. 抓取流程放到后台线程执行，避免在订阅回调里阻塞
3. 增加 /joint_states 就绪门禁，避免机器人状态未准备好就请求 IK
4. 机械臂运动统一走 MoveIt IK + FollowJointTrajectory
5. /grasp_target_pose 使用 VOLATILE 订阅，和 vision_tf_node 保持一致
6. 暂不接入真实灵巧手控制，先保证机械臂主链路跑通
"""

import math
import threading
import time
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes


class GraspControlNode(Node):
    def __init__(self):
        super().__init__('grasp_control_node')

        self.get_logger().info("=" * 60)
        self.get_logger().info("正在初始化抓取控制节点（MoveIt/控制器版本）...")
        self.get_logger().info("=" * 60)

        self.sub_cb_group = ReentrantCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()

        # =========================
        # 参数
        # =========================
        self.declare_parameter('input_topic', '/grasp_target_pose')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('moveit_action_name', '/rm_group_controller/follow_joint_trajectory')
        self.declare_parameter('ik_service_name', '/compute_ik')
        self.declare_parameter('move_group_name', 'rm_group')
        # tcp_link is defined at the dexterous-hand grasp center, not at the arm flange.
        self.declare_parameter('ee_link', 'tcp_link')

        self.declare_parameter('approach_height', 0.10)
        self.declare_parameter('lift_height', 0.15)

        self.declare_parameter('trajectory_duration_sec', 3.0)
        self.declare_parameter('ik_timeout_sec', 1.5)
        self.declare_parameter('future_wait_timeout_sec', 2.0)
        self.declare_parameter('goal_cooldown_sec', 5.0)
        self.declare_parameter('goal_position_threshold', 0.01)

        self.declare_parameter(
            'joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        )

        self.declare_parameter('enable_hand_actions', False)

        self.input_topic = self.get_parameter('input_topic').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.moveit_action_name = self.get_parameter('moveit_action_name').value
        self.ik_service_name = self.get_parameter('ik_service_name').value
        self.move_group_name = self.get_parameter('move_group_name').value
        self.ee_link = self.get_parameter('ee_link').value

        self.approach_height = float(self.get_parameter('approach_height').value)
        self.lift_height = float(self.get_parameter('lift_height').value)
        self.trajectory_duration_sec = float(self.get_parameter('trajectory_duration_sec').value)
        self.ik_timeout_sec = float(self.get_parameter('ik_timeout_sec').value)
        self.future_wait_timeout_sec = float(self.get_parameter('future_wait_timeout_sec').value)
        self.goal_cooldown_sec = float(self.get_parameter('goal_cooldown_sec').value)
        self.goal_position_threshold = float(self.get_parameter('goal_position_threshold').value)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.enable_hand_actions = bool(self.get_parameter('enable_hand_actions').value)

        self.get_logger().info(f"输入话题: {self.input_topic}")
        self.get_logger().info(f"关节状态话题: {self.joint_state_topic}")
        self.get_logger().info(f"控制器 Action: {self.moveit_action_name}")
        self.get_logger().info(f"IK 服务: {self.ik_service_name}")
        self.get_logger().info(f"规划组: {self.move_group_name}")
        self.get_logger().info(f"末端链节: {self.ee_link}")
        self.get_logger().info(f"控制器关节顺序: {self.joint_names}")

        # =========================
        # 状态
        # =========================
        self.is_grasping = False
        self.last_goal_pose: Optional[PoseStamped] = None
        self.last_goal_time_sec: float = 0.0
        self.worker_thread: Optional[threading.Thread] = None
        self.state_lock = threading.Lock()

        self.has_joint_state = False
        self.latest_joint_state: Optional[JointState] = None

        # =========================
        # Action / Service 客户端
        # =========================
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.moveit_action_name,
            callback_group=self.client_cb_group
        )

        self.ik_client = self.create_client(
            GetPositionIK,
            self.ik_service_name,
            callback_group=self.client_cb_group
        )

        self.get_logger().info("Action / IK 客户端已创建，执行抓取时再检查连接状态")

        # =========================
        # 订阅
        # =========================
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.grasp_target_callback,
            pose_qos,
            callback_group=self.sub_cb_group
        )

        joint_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            joint_qos,
            callback_group=self.sub_cb_group
        )

        self.get_logger().info("抓取控制节点初始化完成，等待抓取目标...")

    # =========================
    # Joint States
    # =========================
    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

        if not self.has_joint_state:
            self.has_joint_state = True
            self.get_logger().info(
                f"已收到 joint_states，关节状态就绪，关节名: {list(msg.name)}"
            )

    # =========================
    # 工具函数
    # =========================
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _is_duplicate_goal(self, msg: PoseStamped) -> bool:
        if self.last_goal_pose is None:
            return False

        dt = self._now_sec() - self.last_goal_time_sec
        if dt > self.goal_cooldown_sec:
            return False

        dx = msg.pose.position.x - self.last_goal_pose.pose.position.x
        dy = msg.pose.position.y - self.last_goal_pose.pose.position.y
        dz = msg.pose.position.z - self.last_goal_pose.pose.position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        return distance < self.goal_position_threshold

    def _cache_goal(self, msg: PoseStamped):
        self.last_goal_pose = msg
        self.last_goal_time_sec = self._now_sec()

    def _make_offset_pose(self, pose: PoseStamped, dz: float) -> PoseStamped:
        out = PoseStamped()
        out.header = pose.header
        out.pose.position.x = pose.pose.position.x
        out.pose.position.y = pose.pose.position.y
        out.pose.position.z = pose.pose.position.z + dz
        out.pose.orientation = pose.pose.orientation
        return out

    def _wait_for_future(self, future, timeout_sec: float, description: str):
        start = time.monotonic()
        while rclpy.ok():
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f"{description} 返回异常: {str(e)}")
                    return None

            if time.monotonic() - start > timeout_sec:
                self.get_logger().error(f"{description} 超时（{timeout_sec:.1f}s）")
                return None

            time.sleep(0.05)

        self.get_logger().error(f"{description} 在 ROS 关闭前未完成")
        return None

    # =========================
    # 灵巧手占位动作
    # =========================
    def open_hand(self):
        if self.enable_hand_actions:
            self.get_logger().warning("enable_hand_actions=True，但当前版本未接入真实灵巧手控制。")
        else:
            self.get_logger().info("跳过灵巧手张开：当前版本未启用手部控制。")

    def close_hand(self):
        if self.enable_hand_actions:
            self.get_logger().warning("enable_hand_actions=True，但当前版本未接入真实灵巧手控制。")
        else:
            self.get_logger().info("跳过灵巧手闭合：当前版本未启用手部控制。")

    # =========================
    # IK + 控制器执行
    # =========================
    def solve_ik(self, target_pose: PoseStamped) -> Optional[List[float]]:
        if not self.has_joint_state:
            self.get_logger().error("尚未收到 /joint_states，拒绝执行 IK")
            return None

        if not self.ik_client.service_is_ready():
            self.get_logger().warning("IK 服务未就绪，尝试等待 /compute_ik ...")
            if not self.ik_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error("IK 服务不可用")
                return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.move_group_name
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.pose_stamped = target_pose
        req.ik_request.avoid_collisions = True

        # 使用当前关节状态
        req.ik_request.robot_state.joint_state = self.latest_joint_state
        req.ik_request.robot_state.is_diff = False

        timeout_sec = max(self.ik_timeout_sec, 0.1)
        req.ik_request.timeout.sec = int(timeout_sec)
        req.ik_request.timeout.nanosec = int((timeout_sec - int(timeout_sec)) * 1e9)

        self.get_logger().info("开始调用 /compute_ik ...")
        future = self.ik_client.call_async(req)
        resp = self._wait_for_future(future, self.future_wait_timeout_sec, "/compute_ik")
        self.get_logger().info("/compute_ik 调用结束")

        if resp is None:
            self.get_logger().error("IK 请求失败，未收到响应")
            return None

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"IK 求解失败，MoveIt 错误码: {resp.error_code.val}")
            return None

        name_to_pos = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))

        try:
            ordered_positions = [float(name_to_pos[name]) for name in self.joint_names]
        except KeyError as e:
            self.get_logger().error(
                f"IK 返回的关节名与控制器不一致，缺少关节: {str(e)}；"
                f"IK返回: {list(name_to_pos.keys())}；控制器期望: {self.joint_names}"
            )
            return None

        self.get_logger().info(f"IK 求解成功，目标关节角: {ordered_positions}")
        return ordered_positions

    def execute_joint_positions(self, joint_positions: List[float]) -> bool:
        if not self.trajectory_client.server_is_ready():
            self.get_logger().warning(
                f"轨迹 Action 未就绪，尝试等待 {self.moveit_action_name} ..."
            )
            if not self.trajectory_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().error(
                    f"轨迹 Action 不可用：{self.moveit_action_name}，请检查 action 名称"
                )
                return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(self.trajectory_duration_sec)
        point.time_from_start.nanosec = int(
            (self.trajectory_duration_sec - int(self.trajectory_duration_sec)) * 1e9
        )
        goal.trajectory.points.append(point)

        self.get_logger().info(f"准备发送关节轨迹: {joint_positions}")

        send_future = self.trajectory_client.send_goal_async(
            goal,
            feedback_callback=self.trajectory_feedback_callback
        )
        goal_handle = self._wait_for_future(
            send_future,
            self.future_wait_timeout_sec,
            "send_goal_async"
        )

        if goal_handle is None:
            self.get_logger().error("控制器未返回 goal handle")
            return False

        self.get_logger().info("控制器已返回 goal handle")

        if not goal_handle.accepted:
            self.get_logger().error("轨迹目标被拒绝")
            return False

        result_future = goal_handle.get_result_async()
        result = self._wait_for_future(
            result_future,
            self.future_wait_timeout_sec + self.trajectory_duration_sec,
            "get_result_async"
        )

        if result is None:
            self.get_logger().error("轨迹执行未返回结果")
            return False

        if result.result.error_code == 0:
            self.get_logger().info("轨迹执行成功")
            return True

        self.get_logger().error(
            f"轨迹执行失败，error_code={result.result.error_code}, "
            f"error_string='{result.result.error_string}'"
        )
        return False

    def plan_and_execute_pose(self, target_pose: PoseStamped, stage_name: str) -> bool:
        self.get_logger().info(f"[{stage_name}] 开始 IK 求解与轨迹执行")

        joint_positions = self.solve_ik(target_pose)
        if joint_positions is None:
            self.get_logger().error(f"[{stage_name}] IK 求解失败")
            return False

        ok = self.execute_joint_positions(joint_positions)
        if not ok:
            self.get_logger().error(f"[{stage_name}] 轨迹执行失败")
            return False

        self.get_logger().info(f"[{stage_name}] 执行成功")
        return True

    def trajectory_feedback_callback(self, feedback_msg):
        pass

    # =========================
    # 抓取流程
    # =========================
    def grasp_target_callback(self, msg: PoseStamped):
        if not self.has_joint_state:
            self.get_logger().warning("尚未收到 /joint_states，暂不执行抓取")
            return

        with self.state_lock:
            if self.is_grasping:
                self.get_logger().warning("正在执行抓取流程，忽略新的目标")
                return

            if self._is_duplicate_goal(msg):
                self.get_logger().debug("检测到短时间内重复目标，忽略")
                return

            self._cache_goal(msg)
            self.is_grasping = True

        self.worker_thread = threading.Thread(
            target=self._execute_grasp_flow,
            args=(msg,),
            daemon=True
        )
        self.worker_thread.start()

    def _execute_grasp_flow(self, target_pose: PoseStamped):
        try:
            self.get_logger().info("=" * 60)
            self.get_logger().info("接收到新的抓取目标，开始执行抓取流程")
            self.get_logger().info("=" * 60)

            target_x = target_pose.pose.position.x
            target_y = target_pose.pose.position.y
            target_z = target_pose.pose.position.z

            self.get_logger().info(
                f"目标位姿: ({target_x:.4f}, {target_y:.4f}, {target_z:.4f})"
            )

            approach_pose = self._make_offset_pose(target_pose, self.approach_height)
            lift_pose = self._make_offset_pose(target_pose, self.lift_height)

            self.get_logger().info("\n[步骤 1/5] 张开灵巧手...")
            self.open_hand()

            self.get_logger().info("\n[步骤 2/5] 移动到目标上方...")
            if not self.plan_and_execute_pose(approach_pose, "approach"):
                return

            self.get_logger().info("\n[步骤 3/5] 向下移动到目标位置...")
            if not self.plan_and_execute_pose(target_pose, "descend"):
                return

            self.get_logger().info("\n[步骤 4/5] 闭合灵巧手抓取...")
            self.close_hand()

            self.get_logger().info("\n[步骤 5/5] 抬起机械臂...")
            if not self.plan_and_execute_pose(lift_pose, "lift"):
                self.get_logger().warning("抬升可能失败，但前面步骤已执行")

            self.get_logger().info("=" * 60)
            self.get_logger().info("抓取流程执行完成")
            self.get_logger().info("=" * 60)

        except Exception as e:
            self.get_logger().error(f"抓取过程中发生错误: {str(e)}")
        finally:
            with self.state_lock:
                self.is_grasping = False

    def destroy_node(self):
        try:
            self.get_logger().info("正在关闭抓取控制节点...")
        except Exception:
            pass

        super().destroy_node()

        try:
            self.get_logger().info("抓取控制节点已关闭")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)

    try:
        node = GraspControlNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        try:
            executor.shutdown()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
