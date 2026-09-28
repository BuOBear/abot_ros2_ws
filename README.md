# ABOT ROS 2 Humble

统一工作区：`/home/manbo_u22/abot_ros2_ws`。原 ABOT 与机械臂工程已合并，ROS1 源码已移除；`src/` 包含 15 个项目包。目标环境为 Ubuntu 22.04、ROS 2 Humble、系统 Python 3.10 和 C++17。

合并并更名后已完成 15 包构建、247 项测试和安装态默认启动/组合建图检查。实车底盘、导航、腕部标定及自动抓取尚未验收；原比赛 YOLO11 权重已丢失。历史严格 P0 测试与当前验证范围见下方验收文档。

## 文档导航

| 文档 | 内容 |
|---|---|
| [迁移状态](docs/MIGRATION_STATUS.md) | 包职责、完成范围、剩余工作与历史证据位置 |
| [部署与依赖](docs/DEPLOYMENT.md) | 目标主机、设备盘点、依赖安装及 WSL 本地叠加层 |
| [接口约定](docs/INTERFACES.md) | 话题、服务、坐标系、控制权与发布职责 |
| [底盘与导航](docs/BASE.md) | 串口协议、生命周期、机器人模型、定位导航及速度链 |
| [视觉与跟随](docs/VISION.md) | 检测器、模板资源、视觉/雷达跟随及 ROI 跟踪 |
| [地图与任务](docs/TASKS.md) | 比赛地图、路线、附件、语音与 VLM |
| [机械臂与腕部视觉](docs/MOBILE_MANIPULATOR.md) | 舵机板、分阶段启动、YOLO11 与平面估计 |
| [验证与验收](docs/VALIDATION.md) | 测试证据、复现命令、严格离线规则及实车清单 |

## 构建与启动

按部署文档安装依赖并加载 ROS 环境，在工作区根目录执行：

```bash
bash tools/verify.sh
source install/setup.bash
ros2 launch abot_bringup bringup.launch.py
```

默认只发布机器人模型；硬件和传感器需显式启用。移动机械臂使用下面的组合入口，它管理同一组底盘节点，因此两种入口只选一个：

```bash
ros2 launch jy_real_robot real.launch.py
```

底盘、EKF 和导航保留横向速度，最终 `/cmd_vel` 由 `velocity_gate` 唯一发布。底盘入口沿用旧 640×480 CameraInfo 作为待实测候选；机械臂入口不沿用旧固定相机 TF，腕部 CameraInfo 默认空，需重新确认标定。

地图仅保留 `robot_slam/1` 与 `robot_slam/shoot` 两组 PGM/YAML；三张场地示意图只供参考。三场 2025 年比赛尚未分配地图、脚本和路线，任务不会自动执行。

源码位于 `src/`，维护文档位于 `docs/`，部署配置位于 `deployment/`，验证工具位于 `tools/`；本地构建和测试证据位于 `build/`、`install/`、`log/`、`.validation/`。
