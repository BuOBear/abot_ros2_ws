# ROS 2 集成接口约定

目标环境：Ubuntu 22.04、ROS 2 Humble、C++17、系统 Python 3.10。只构建当前工作区的 `src`，ROS1 源码已移除。

## 发布职责与坐标系

* `abot_hardware`：生命周期程序 `base_driver`，节点 `abot_hardware`，独占串口通信，默认未激活。不发布里程计 TF，不主动写入固件参数。激活需有效连接；命令使用单调时钟超时；停用时输出零，重连后不重放过期命令。
* `abot_description`：`description.launch.py`，参数 `use_sim_time`；根为 `base_footprint`，固定坐标系包括 `base_link`、`laser_link`、`imu_link`、`camera_link`、`camera_optical_frame`；轮关节保持真实状态语义。
* `abot_navigation`：`localization.launch.py`（EKF/IMU）、`navigation.launch.py`（`mode:=mapping|localization`、`map`、`use_sim_time`）、`velocity.launch.py`（唯一速度链）。仅 EKF 发布 `odom -> base_footprint`；仅 AMCL 或 SLAM 发布 `map -> odom`。
* `abot_bringup`：顶层启动、设备配置与传感器封装，只引用已安装包共享目录。在 M0 硬件盘点确认前，设备必须显式启用。
* `/control/mode`（`std_msgs/msg/String`，可靠/易失性，深度 1）显式选择 `disabled`、`nav`、`tracking`、`teleop`，初始关闭。源超时后在当前模式保持零，不能恢复其他源。切换自主模式前，管理器必须取消并确认当前 Nav2 目标；仅发送 disabled 不会取消目标。
* `/control/mode_state`（`std_msgs/msg/String`，可靠/瞬态本地，深度 1）报告初始 `disabled`，回显每个有效模式命令，包括重复值。`/control/mode_epoch`（`std_msgs/msg/UInt64`，相同 QoS）从 0 开始，每次有效转换递增。无效模式字符串不回显。任何外部模式命令（含重复 `nav`）均撤销导航租约。
* `/control/acquire_nav` 和 `/control/release_nav` 为有类型定义的 `abot_control_interfaces` 服务。Acquire 只将 `disabled` 改为 `nav`，返回非零租约 ID 与代次；Release 只将当前属于此租约的 `nav` 改为 `disabled`。旧释放请求不能覆盖后来的 `teleop`、`tracking` 或外部选择的 `nav`。这些回调与模式订阅共享互斥回调组。
* 速度授权还要求新鲜滤后扫描和可用里程计/传感器 TF。扫描与动态 TF 时间戳具有有界单调有效期，重复时间戳不刷新有效期。缺扫描或 TF 缺失/过期时，在进入 mux/平滑器/监测器链前，所有模式（含遥控）都保持零。

P1 视觉模块加入 `abot_perception`，提供带时间戳的颜色、标签、模板和完整人体检测，并加入 `abot_tracking` 进行仅偏航对齐。跟随器只向已有跟随命令源发布，不选择控制模式，不从未标定像素框推算距离。选择视觉跟随时，启动文件选择一个检测源及类别/ID。模板检测器最多接受 8 个数字图像 ID，跟随器仍只选择其中一个。运行时 `/cmd_vel/tracking` 最多一个发布者。可选 P2 人脸源在 `/perception/faces` 发布通用二维框，不识别身份；来自 color_pkg 的 `/perception/fire` 只在配置的暖色轮廓门槛通过时发布暂定 `fire_candidate`，不触发语音或执行器。可选雷达跟随器用连续两帧扫描佐证前方最近回波，再向同一 `/cmd_vel/tracking` 发布；不识别人、不选择控制模式，线性运动默认关闭。同一时间只运行一个跟随发布者。可选 ROI LK 节点接受一个带时间戳的 `roi_seed` Detection2DArray，要求坐标系和时间戳与相机图像完全匹配；在 `/roi_lk_tracker/roi` 输出带时间戳 `roi` 框，处理图像时跟踪丢失则发空数组。图像流停止由下游新鲜度超时处理。ROI 跟踪本身不发布速度，`roi_track.launch.py` 只有在显式覆盖参数后才启用跟随器，要求同时间戳 CameraInfo，并遵守单发布者规则。

`abot_mission` 只在 `/mission/start` 后执行配置地图路线。每个 Nav2 目标前要求 ACTIVE 的 `bt_navigator`、新鲜 `map -> base_footprint` TF、成功获取导航租约及匹配的 `nav` 模式/代次。结果与取消请求绑定当前 NavigateToPose 目标句柄，只有 SUCCEEDED 才推进步骤。可选视觉观测为被动模式，要求到点后收到新鲜、类别匹配的 `Detection2DArray`。示例路线不含可用坐标，不调用附件，也不启动跟随。

`abot_accessory` 独占另一个可选 9600 波特率串口，暴露显式 `/accessory/shoot`、`/accessory/stop` Trigger 服务；初始关闭，启动不发帧。`abot_voice` 在 `/voice/transcribe/request` 接受受限 JSON 文件请求，在 `/voice/transcribe/status` 按请求 ID 报告 JSON 状态，在 `/voice/text` 发布识别文本。后端需另行提供本地模型，不采集音频、不选择控制模式。

`abot_vlm` 将 `/vlm/analyze` 暴露为 `abot_vlm_interfaces/action/AnalyzeImage` Action。每个显式目标截取一张近期 `/camera/image_raw` 帧，编码及可选远程推理在 ROS 图像回调之外执行。结果代码区分缺帧/过期帧、后端失败、超时和取消。同一时间只接纳一个后端请求，默认启动不发送请求。

## 话题

| 名称 | 类型 | 约定 |
|-|-|-|
| `/wheel_odom` | nav_msgs/msg/Odometry | 可靠、易失性，深度 10；odom/base_footprint；只发新反馈；国际单位制，包含 vy |
| `/imu/data_raw` | sensor_msgs/msg/Imu | SensorDataQoS；imu_link；国际单位制；orientation_covariance[0] = -1 |
| `/imu/data` | sensor_msgs/msg/Imu | madgwick 输出；订阅者需配置兼容 QoS |
| `/imu/mag` | sensor_msgs/msg/MagneticField | 可选，单位特斯拉；初始不融合 |
| `/odom` | nav_msgs/msg/Odometry | EKF 的 odometry/filtered 重映射到此处 |
| `/scan` -> `/scan_filtered` | sensor_msgs/msg/LaserScan | laser_link；传感器兼容 QoS |
| `/cmd_vel/nav`, `/cmd_vel/tracking`, `/cmd_vel/teleop` | geometry_msgs/msg/Twist | 可靠/易失性；有效期有界；源选择须防止过期源恢复 |
| `/cmd_vel` | geometry_msgs/msg/Twist | 仅 `velocity_gate` 发布最终命令；硬件订阅深度 1 |
| `/camera/image_raw`, `/camera/camera_info` | sensor_msgs 图像接口 | 物理相机单一所有者，时间戳/坐标系匹配 |
| `/perception/colors`, `/perception/tags`, `/perception/templates`, `/perception/people`, `/perception/faces`, `/perception/fire` | vision_msgs/msg/Detection2DArray | 传感器兼容 QoS；每话题一个源，像素框，原图时间戳/坐标系；空数组清该源目标；人脸无身份标签，火焰仅为颜色/轮廓候选 |
| `/roi_lk_tracker/roi_seed` | vision_msgs/msg/Detection2DArray | 可靠/易失性，深度 1；恰好一个 `roi_seed` 假设，轴对齐像素框，与种子图像相同非零时间戳及光学坐标系 |
| `/roi_lk_tracker/roi` | vision_msgs/msg/Detection2DArray | 可靠/易失性，深度 1；原图时间戳/坐标系；跟踪时一个 `roi` 框，处理图像时丢失则为空数组；无心跳 |
| 来自 `abot_tracking` 的 `/cmd_vel/tracking` | geometry_msgs/msg/Twist | 可靠/易失性，深度 1；同一时间选一个视觉或雷达跟随器；目标丢失/输入过期产生新零命令；仅显式 tracking 模式下有效；雷达线速度需显式覆盖 |
| `/mission/start`, `/mission/cancel` | std_srvs/srv/Trigger | 显式启动/取消路线；`/mission/status` 为易失性 JSON String 状态 |
| `/accessory/shoot`, `/accessory/stop` | std_srvs/srv/Trigger | 设备适配器默认关闭；响应报告写入结果，不代表实物执行结果 |
| `/voice/transcribe/request` | std_msgs/msg/String | JSON `{id,file}`；配置收件目录下已写完的 PCM WAV 基本文件名 |
| `/voice/transcribe/status`, `/voice/text` | std_msgs/msg/String | JSON 状态含请求 ID；文本为不带关联信息的纯文本流 |
| `/control/mode_epoch` | std_msgs/msg/UInt64 | 可靠/瞬态本地，深度 1；每个有效模式转换递增，包括外部重复值 |
| `/control/acquire_nav`, `/control/release_nav` | abot_control_interfaces 服务 | 租约 ID 与代次防止任务获取/释放覆盖后续控制变化 |
| `/vlm/analyze` | abot_vlm_interfaces/action/AnalyzeImage | 显式、可取消图像分析；受帧与后端限制 |

## 范围与阶段门槛

实现覆盖离线 M1、可测试 M2/P0、受限 P1 视觉/任务/附件/语音/VLM 模块，以及部分离线 P2 人脸/火焰/雷达/ROI 路径。四种请求的视觉目标均有离线二维实现，但实体标签格式和实际模板/标记 ID 尚未确认。M0 实物信息和 M2/M3 真机检查仍是现场部署的明确前置条件，离线进度不能豁免。旧尺寸、传感器变换、尺度约定、地图和 HSV 阈值是候选值，不是实测标定。

<a id="leases"></a>

## 导航租约服务细节

`/control/acquire_nav` 接受空的 `abot_control_interfaces/srv/AcquireNav` 请求。控制权管理器只在当前模式为 `disabled` 时授予请求，在同一个串行化回调中切换到 `nav`，并返回非零 `lease_id` 和新的 `epoch`。拒绝请求时返回 `success=false`、`lease_id=0` 和当前代次。

`/control/release_nav` 接受携带该租约 ID 的 `abot_control_interfaces/srv/ReleaseNav` 请求。仅当这个 ID 仍持有当前 `nav` 模式时，才切换到 `disabled`；被拒绝的释放请求不会改变模式。

每个有效的外部 `/control/mode` 命令（包括重复的 `nav`）都会撤销租约并推进 `/control/mode_epoch`。成功获取和释放租约也会推进代次。代次话题类型为 `std_msgs/msg/UInt64`，使用可靠、瞬态本地（transient-local）QoS，深度为 1，初值为 0。租约 ID 来自系统随机源，并不是授权凭证：任何能够访问这些话题和服务的 ROS 参与者都可以改变控制模式。需要访问限制时，应另行部署 DDS 访问控制。

## 机械臂与腕部视觉接口

移动机械臂入口 `jy_real_robot real.launch.py` 复用底盘节点，并关闭原固定相机 TF；其腕部光学帧为 `wrist_camera_optical_frame`。没有实测关节反馈和手眼标定时，不发布虚构的动态腕部 TF 或 `/joint_states`。

| 接口 | 类型 | 约定 |
|---|---|---|
| `/arm/set_servo_angle` | `jy_real_interfaces/srv/SetServoAngle` | 请求 `actuator`、整数 `board_angle_deg`；`sent_unverified` 只说明主机串口写入，不证明执行 |
| `/perception/plane_point` | `geometry_msgs/msg/PointStamped` | 可选已知平面交点，按原图时刻查询可信 TF，无运动输出 |

YOLO11 输出标准 `vision_msgs/msg/Detection2DArray`；权重、标定、设备与启动参数见 [机械臂与腕部视觉](MOBILE_MANIPULATOR.md)。
