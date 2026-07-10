# 问题解决记录：从“节点都在”到“系统真的动起来”

我在这个项目里学到的最重要经验是：机器人系统报错的位置，常常不是问题真正发生的位置。排查时要沿 **模型 → TF → 状态 → 规划 → 控制器** 逐层验证。

## 1. `robot_description` 为空

**现象**：RViz 某些节点能看到模型，但 `move_group` 报 robot model 不存在。

**定位**：不同 Launch 文件各自持有参数；“某个节点有 URDF”不等于 MoveIt 也收到了 URDF。

**解决**：使用 `MoveItConfigsBuilder` 显式加载合并后的 `rm65_with_hand.urdf.xacro` 和匹配的 SRDF，再把 `moveit_config.to_dict()` 直接交给 `move_group`。

**可迁移经验**：遇到参数问题先执行 `ros2 param get /move_group robot_description`，不要只凭 RViz 画面判断。

## 2. `Unable to parse robot_description as yaml`

**现象**：Launch 参数包含 XML 字符串，却被 ROS 2 当成 YAML 解析。

**根因**：Launch substitution 的类型没有被固定。

**解决**：需要传字符串的动态参数使用 `ParameterValue(..., value_type=str)`。同时区分“文件路径”“文件内容”和“Launch substitution”，三者不能混用。

## 3. 合并模型后规划组不一致

**现象**：URDF 已经包含机械臂和手，但 IK 的末端仍停在 `Link6`，抓取点有固定偏差。

**解决**：添加 `tcp_link`，并把 SRDF 的 `rm_group` 改为 `base_link → tcp_link`。检查 URDF link、SRDF group、IK `ik_link_name` 三处名称完全一致。

**可迁移经验**：URDF 描述“机器人是什么”，SRDF 描述“规划器如何使用机器人”；修改一个通常需要同步检查另一个。

## 4. 有检测结果，但机械臂不动作

我没有直接把它归因于“MoveIt 坏了”，而是拆成五个门：

1. `/yolo/detections_3d` 是否持续发布且 `bbox3d` 有效；
2. `camera_link → base_link` TF 是否存在；
3. `/grasp_target_pose` 是否发布；
4. `/compute_ik` 是否返回成功错误码；
5. `/rm_group_controller/follow_joint_trajectory` 是否存在并接受目标。

最终代码加入 `/joint_states` 就绪门禁、Service/Action 可用性检查、IK 错误码和关节名对照日志。这样失败时能够知道链路断在哪一层。

## 5. 节点初始化或回调“卡死”

**根因**：在 `__init__` 或订阅回调中同步等待 Service/Action，会占住执行器线程；响应回调得不到调度，形成看似玄学的等待。

**解决**：

- 使用 `MultiThreadedExecutor` 与 `ReentrantCallbackGroup`；
- 抓取状态机放到后台线程；
- 所有 Future 都带超时；
- 新目标到达时使用状态锁避免并发执行两次抓取。

## 6. QoS 看不见，但确实能让消息消失

发布端与订阅端的可靠性、持久性不兼容时，话题名称完全正确也可能收不到数据。我将 `/grasp_target_pose` 两端统一为 `RELIABLE + VOLATILE`，并用 `ros2 topic info -v` 检查端点 QoS。

## 7. 为什么不上传整个 200 MB 工作空间？

原工作空间包含 `build/`、`install/`、`log/`、重复的嵌套工作空间、第三方 `.git` 历史和 120+ MB 的模型权重。它们会掩盖真正的工程贡献，还会携带本机绝对路径。

本仓库只保留项目集成层、关键配置和文档；厂商驱动、模型网格和权重通过上游依赖获取。这个取舍让仓库从“电脑备份”变成“可阅读的工程作品”。

## 我的通用排查顺序

```text
消息有数据吗？
  → frame_id 和 TF 对吗？
    → robot_description / SRDF 一致吗？
      → joint_states 就绪吗？
        → IK 成功吗？
          → controller Action 存在并接收目标吗？
```

这个顺序帮助我把跨模块问题转化为一组可以逐个证伪的小问题。
