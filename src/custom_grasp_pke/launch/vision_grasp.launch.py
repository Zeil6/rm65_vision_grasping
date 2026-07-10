#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ==============================
    # 启动参数
    # ==============================
    declare_eye_in_hand_x = DeclareLaunchArgument(
        'eye_in_hand_x',
        default_value='-0.09649021',
        description='相机相对于机械臂末端的X方向偏移（米）'
    )
    declare_eye_in_hand_y = DeclareLaunchArgument(
        'eye_in_hand_y',
        default_value='0.03243441',
        description='相机相对于机械臂末端的Y方向偏移（米）'
    )
    declare_eye_in_hand_z = DeclareLaunchArgument(
        'eye_in_hand_z',
        default_value='0.03628098',
        description='相机相对于机械臂末端的Z方向偏移（米）'
    )
    declare_eye_in_hand_qx = DeclareLaunchArgument(
        'eye_in_hand_qx',
        default_value='0.49395403',
        description='相机相对于机械臂末端的四元数X分量'
    )
    declare_eye_in_hand_qy = DeclareLaunchArgument(
        'eye_in_hand_qy',
        default_value='0.49657568',
        description='相机相对于机械臂末端的四元数Y分量'
    )
    declare_eye_in_hand_qz = DeclareLaunchArgument(
        'eye_in_hand_qz',
        default_value='-0.48568989',
        description='相机相对于机械臂末端的四元数Z分量'
    )
    declare_eye_in_hand_qw = DeclareLaunchArgument(
        'eye_in_hand_qw',
        default_value='0.52299841',
        description='相机相对于机械臂末端的四元数W分量'
    )

    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='机械臂基座坐标系名称'
    )
    declare_tool_frame = DeclareLaunchArgument(
        'tool_frame',
        default_value='Link6',
        description='机械臂末端坐标系名称'
    )

    eye_in_hand_x = LaunchConfiguration('eye_in_hand_x')
    eye_in_hand_y = LaunchConfiguration('eye_in_hand_y')
    eye_in_hand_z = LaunchConfiguration('eye_in_hand_z')
    eye_in_hand_qx = LaunchConfiguration('eye_in_hand_qx')
    eye_in_hand_qy = LaunchConfiguration('eye_in_hand_qy')
    eye_in_hand_qz = LaunchConfiguration('eye_in_hand_qz')
    eye_in_hand_qw = LaunchConfiguration('eye_in_hand_qw')

    base_frame = LaunchConfiguration('base_frame')
    tool_frame = LaunchConfiguration('tool_frame')

    # ==============================
    # 1) 机械臂本体 bringup
    # 直接按你给的 rm_bringup 内容展开
    # ==============================
    rm_65_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_driver'),
                'launch',
                'rm_65_driver.launch.py'
            )
        )
    )

    rm_65_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_description'),
                'launch',
                'rm_65_6f_display.launch.py'
            )
        )
    )

    rm_65_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_control'),
                'launch',
                'rm_65_control.launch.py'
            )
        )
    )

    rm_65_moveit_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_65_config'),
                'launch',
                'real_moveit_demo_6f.launch.py'
            )
        )
    )

    # ==============================
    # 2) 手眼静态 TF：末端 -> 相机
    # ==============================
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='eye_in_hand_static_tf',
        output='screen',
        arguments=[
            eye_in_hand_x,
            eye_in_hand_y,
            eye_in_hand_z,
            eye_in_hand_qx,
            eye_in_hand_qy,
            eye_in_hand_qz,
            eye_in_hand_qw,
            tool_frame,
            'camera_link'
        ]
    )

    # ==============================
    # 3) 视觉 TF 转换节点：camera_link -> base_frame
    # ==============================
    vision_tf_node = Node(
        package='custom_grasp_pke',
        executable='vision_tf_node',
        name='vision_tf_node',
        output='screen',
        parameters=[
            {
                'input_topic': '/yolo/detections_3d',
                'output_topic': '/grasp_target_pose',
                'source_frame': 'camera_link',
                'target_frame': base_frame,
            }
        ]
    )

    # ==============================
    # 4) 抓取控制节点
    # ==============================
    grasp_control_node = Node(
        package='custom_grasp_pke',
        executable='grasp_control_node',
        name='grasp_control_node',
        output='screen',
        parameters=[
            {
                'input_topic': '/grasp_target_pose',
                'approach_height': 0.10,
                'lift_height': 0.15,
                'ee_link': 'tcp_link',
                'moveit_action_name': '/rm_group_controller/follow_joint_trajectory',
            }
        ]
    )

    # ==============================
    # LaunchDescription
    # ==============================
    ld = LaunchDescription()

    ld.add_action(declare_eye_in_hand_x)
    ld.add_action(declare_eye_in_hand_y)
    ld.add_action(declare_eye_in_hand_z)
    ld.add_action(declare_eye_in_hand_qx)
    ld.add_action(declare_eye_in_hand_qy)
    ld.add_action(declare_eye_in_hand_qz)
    ld.add_action(declare_eye_in_hand_qw)
    ld.add_action(declare_base_frame)
    ld.add_action(declare_tool_frame)

    ld.add_action(LogInfo(msg='========================================'))
    ld.add_action(LogInfo(msg='正在启动眼在手上机械臂抓取系统...'))
    ld.add_action(LogInfo(msg='1. 启动 RM 机械臂 driver/description/control/moveit'))
    ld.add_action(LogInfo(msg='2. 发布手眼静态 TF: tool_frame -> camera_link'))
    ld.add_action(LogInfo(msg='3. 启动 vision_tf_node 与 grasp_control_node'))
    ld.add_action(LogInfo(msg='========================================'))

    # 先启动机械臂本体相关节点
    ld.add_action(rm_65_driver)
    ld.add_action(rm_65_description)
    ld.add_action(rm_65_control)
    ld.add_action(rm_65_moveit_config)

    # 再启动你自己的视觉抓取链
    ld.add_action(static_tf_node)
    ld.add_action(vision_tf_node)
    ld.add_action(grasp_control_node)

    return ld
