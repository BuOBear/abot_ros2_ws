# 迁移状态

当前唯一工作区为 `/home/manbo_u22/abot_ros2_ws`，已完成原 ABOT 与 catkin 工程的 ROS2 源码合并、目录更名和 ROS1 源码移除。`src/` 包含 15 个项目包；不再依赖旧工作区，也没有 `legacy/` 快照目录。维护入口为根目录 README，其余文档按主题集中于 `docs/`。

## 已完成的软件范围

| 模块 | 包 | 当前结果 |
|---|---|---|
| 底盘与组合启动 | `abot_hardware`、`abot_description`、`abot_navigation`、`abot_bringup` | 生命周期串口、协议与看门狗、模型、IMU/EKF、SLAM/AMCL、全向 Nav2 和最终速度门控；P0 代码及历史严格离线验收完成 |
| 地图 | `abot_maps` | 仅两组 `robot_slam/1`、`robot_slam/shoot` PGM/YAML，以及三张参考场地图 |
| 视觉与跟随 | `abot_perception`、`abot_tracking` | 颜色、标签、模板、人体、人脸框、火焰候选，以及视觉/雷达对齐与种子 ROI LK 观测 |
| 任务与控制接口 | `abot_mission`、`abot_control_interfaces` | 显式地图路线、精确 Nav2 目标结果、导航租约和到点后被动观测 |
| 扩展模块 | `abot_accessory`、`abot_voice`、`abot_vlm_interfaces`、`abot_vlm` | 显式附件服务、本地 WAV 识别/可选播报、按需 VLM Action |
| 机械臂与腕部视觉 | `jy_real_interfaces`、`jy_real_robot` | 组合启动、9600 波特率舵机板服务、YOLO11 接口及可选平面交点；默认关闭设备 |

原 catkin 中可复用的底盘/导航能力统一由 ABOT 提供，旧 YOLO5/自定义检测消息由 YOLO11 接口与标准 `vision_msgs` 替代。RM65、Gazebo、D435 和 MoveIt 1 仿真配置不适用于当前五自由度真机，未迁入运行栈。

## 验证结论

合并并更名后，15 个包构建完成，**247 项测试通过，错误、失败、跳过均为零**；默认无设备启动及组合建图检查通过。此前 P0 在 13 包/202 项测试基线上，严格集成测试在两个全新 ROS 域连续通过。合并后的组合建图检查范围较窄，不能视为重新执行了完整 P0 双次严格验收。证据、环境与复现命令统一见 [验证与验收](VALIDATION.md)。

## 尚未完成的工作

1. **实车盘点与底盘验收：** 设备唯一标识、固件基线、轮组几何/方向、IMU 与传感器外参、MCU 独立停车时间。
2. **实车导航：** 实测地图保存/重载、定位、到点成功、取消/恢复、多目标与重复成功率。离线 Nav2 命令传播和取消不等于真机到点。
3. **视觉与任务：** 实体标签兼容性、模板 ID、真实数据准确率、音频与远程 VLM；2025 年三场赛事的地图、脚本和路线仍未分配。3×1.5 仅保留示意图，不计划恢复第三张地图。
4. **机械臂：** 通道/限位、可信关节反馈、尺寸与碰撞模型、腕部标定、动作互锁与执行结果。原比赛 YOLO11 权重已丢失，真实推理及自动抓取未验收。

硬件、EKF 和 DWB 均保留横向速度。最终 `/cmd_vel` 只由 `velocity_gate` 发布；无时间戳 Twist 的恢复保证基于接收回调和缓存新鲜度，不能证明延迟命令的原始发布时间。

## 文档与历史证据

旧规划、交接、审查续篇、逐包 README 和重复迁移记录已合并为当前主题文档；不再维护多个状态口径。日志继续保存在 `.validation/`。原 ROS1 校验清单和目录重组清单移至 `.validation/history/ROS1_BASELINE.json`、`.validation/history/RESTRUCTURE_MANIFEST.json`，只用于追溯，清单中的旧路径不代表文件仍存在。这些本地证据未必随 Git 分发，重新验收须生成新日志。
