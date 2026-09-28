# 底盘、模型与导航

部署依赖见 [部署说明](DEPLOYMENT.md)，接口职责见 [接口约定](INTERFACES.md)。

<a id="hardware"></a>

## 底盘串口与生命周期

`base_driver` 是名为 `abot_hardware` 的生命周期节点。用户报告底盘控制器为 STM32F103VET6，通过 USB 转 TTL 连接。固件已烧录到 MCU，其源码可能丢失。芯片型号不能证明已烧录固件的协议或适配器端口；连接这套源自 ROS1 的协议实现前，需确认两者。节点启动时为未配置状态，执行 `configure` 后进入未激活状态。`activate` 会打开并锁定串口设备，要求收到有效固件版本帧，并在接受新命令前发送零速度。`deactivate`、`shutdown` 和激活期间断开时，只要设备仍可访问，就尝试发送零速度。激活期间断开会按有界串口超时重试，重连后必须收到新的 `/cmd_vel` 才恢复。不发送固件配置或里程计复位消息。`/dev/abot` 应由本节点独占，旧 ROS 1 驱动必须停止。

```
ros2 run abot_hardware base_driver --ros-args -p port:=/dev/abot
ros2 lifecycle set /abot_hardware configure
ros2 lifecycle set /abot_hardware activate
```

默认话题：`/cmd_vel`（`geometry_msgs/msg/Twist`，可靠，深度 1）、`/wheel_odom`（`nav_msgs/msg/Odometry`，可靠，深度 10）、`/imu/data_raw`（`sensor_msgs/msg/Imu`，SensorDataQoS）、可选 `/imu/mag`（`sensor_msgs/msg/MagneticField`，SensorDataQoS）及 `/diagnostics`（`diagnostic_msgs/msg/DiagnosticArray`）。本节点不发布 TF。里程计坐标系默认为 `odom` 和 `base_footprint`，IMU 坐标系默认为 `imu_link`。只有对应查询返回完整有效响应时才发布反馈。时间戳采用主机接收时间，因为固件协议没有设备时间戳。

配置完成后参数不可修改。物理硬件要求在配置阶段及运行期间始终使用 `use_sim_time=false`：

| 参数 | 默认值 | 约束/含义 |
| --- | --- | --- |
| `port` | `/dev/abot` | 串口绝对路径 |
| `baud_rate` | `921600` | 115200、230400、460800、500000、576000、921600 或 1000000 |
| `command_timeout_ms` | `250` | 使用单调时钟，范围为命令周期至 2000 ms |
| `response_timeout_ms` | `100` | 每次串口事务 10–1000 ms |
| `reconnect_delay_ms` | `500` | 100–10000 ms |
| `command_period_ms` | `50` | 10–1000 ms |
| `odom_period_ms` | `50` | 10–1000 ms |
| `imu_period_ms` | `20` | 10–1000 ms |
| `publish_imu` | `true` | 查询并发布 IMU |
| `publish_mag` | `false` | 以特斯拉发布原始磁力计数据；融合前需标定 |
| `odom_frame`, `base_frame`, `imu_frame` | `odom`, `base_footprint`, `imu_link` | 非空且不以斜杠开头的 TF 名称 |
| `max_vx_mps`, `max_vy_mps`, `max_wz_radps` | `2`, `2`, `5` | 有限正数输入限制；超限命令被拒绝并输出零 |
| `odom_pose_x_stddev_m`, `odom_pose_y_stddev_m`, `odom_pose_yaw_stddev_rad` | `1`, `1`, `1` | 暂定姿态噪声；前向与横向运动分别调参 |
| `odom_twist_vx_stddev_mps`, `odom_twist_vy_stddev_mps`, `odom_twist_wz_stddev_radps` | `0.1`, `0.1`, `0.2` | 暂定速度噪声；横向打滑单独调参 |
| `imu_accel_stddev_mps2`, `imu_gyro_stddev_radps`, `imu_mag_stddev_tesla` | `0.5`, `0.1`, `1e-5` | 暂定国际单位制噪声；需实车验证 |

协议来自 ROS 1 `abot_bringup` 的 `simple_dataframe.h`、`dataframe.h`、`data_holder.h` 和 `base_driver.cpp`：`0x5a`、单字节 ID、单字节载荷长度、载荷，以及模 256 加和校验。多字节值采用小端。速度命令为三个有符号 16 位百分之一单位值（x/y 为 cm/s，偏航为 0.01 rad/s）；里程计包含这三项速度、以 cm 表示的有符号 32 位 x/y，以及以 0.01 rad 表示的有符号 16 位偏航角。九个 IMU 字段是小端 IEEE-754 binary32。旧 `abot_imu` 桥接节点将前六项不经缩放传给加速度和角速度，将后三项视为毫高斯（每单位 `1e-7` 特斯拉）。姿态未知（`orientation_covariance[0] = -1`）。旧桥接节点的静态偏置、实物 IMU 轴向、固件边界上的加速度/陀螺仪单位、磁力计标定、里程计尺度和协方差，均需 M0/M2 台架及实车测量。

固件响应不含序号。驱动每次请求前清空待处理输入，检查 ID、精确长度、校验和及有界截止时间；超时时不会重发上一帧反馈。如果旧响应延迟到同 ID 的新请求之后才到达，协议无法将其与新响应区分；接受真机导航前须测量固件响应时序。用户报告主机断电或串口丢失时 MCU 会立即停电机。应在静止台架上测量这一停车时间；主机命令超时和重复发送零帧不能量化板卡的独立响应。

命令看门狗使用单调时钟，在串口反馈事务之间检查命令是否超时。端口正常响应时，最后一条命令之后，首个零帧开始写入的时间约不晚于 `command_timeout_ms + command_period_ms + response_timeout_ms`；零帧写入本身最多还需一个 `response_timeout_ms`。这只是调度上界，不能证明板卡已收到或执行该帧。端口断开或阻塞时，无法保证主机驱动的停车；必须实测用户报告的 MCU 独立停车响应。

<a id="description"></a>

## 机器人模型与固定坐标系

`description.launch.py` 通过 `robot_state_publisher` 发布已安装的 `abot.urdf.xacro`，接受 `use_sim_time` 参数（默认 `false`）。固定树为 `base_footprint -> base_link -> {laser_link, imu_link, camera_link}` 和 `camera_link -> camera_optical_frame`。四个轮关节均为连续关节。不启动关节状态发布器：只有数据源使用 URDF 中的关节名发布实际关节位置时，轮坐标系才会出现。用于估计的轮式里程计来自硬件反馈链路，不来自此模型。

网格几何和坐标变换来自 ROS 1 `abot_model` URDF，均为**未经实测的候选值**。尤其是，继承的 `base_footprint -> base_link` 的 z 偏移为零，但 CAD 轮网格延伸到了底盘网格下方。在将几何用于净空判断或标定前，应测量离地高度、轮心、轮序与正负方向、相机朝向和传感器安装位置。旧 ROS 1 启动文件还存在冲突的 IMU TF 发布者，不应与此描述同时运行。本包不包含仿真插件或动力学模型。

<a id="navigation"></a>

## 状态估计、导航与速度链

本包负责局部状态估计、SLAM 或 AMCL、Nav2，以及唯一的速度处理链。`abot_description` 和 `/scan_filtered` 发布者需单独启动。硬件驱动发布 `/wheel_odom` 与 `/imu/data_raw`，**不得**发布里程计 TF。只有 EKF 发布 `odom -> base_footprint`；建图模式只有 slam_toolbox、定位模式只有 AMCL 发布 `map -> odom`。只运行一个 `navigation.launch.py` 实例。

```bash
ros2 launch abot_navigation localization.launch.py use_sim_time:=false
ros2 launch abot_navigation navigation.launch.py mode:=mapping use_sim_time:=false
# 或使用已有且经过验证的地图 YAML：
ros2 launch abot_navigation navigation.launch.py mode:=localization map:=/absolute/map.yaml use_sim_time:=false
ros2 launch abot_navigation velocity.launch.py use_sim_time:=false
```

速度链为 `/cmd_vel/nav|tracking|teleop` → 显式控制权管理 → `twist_mux` → `velocity_smoother` → `collision_monitor` → `velocity_gate` → `/cmd_vel`。`velocity_gate` 是唯一最终发布者；监测器输出、雷达扫描或里程计 TF 过期时，每 50 ms 发出一个新的零命令。碰撞监测器保留机器人移动时采集雷达数据所需的底盘位移修正。Nav2 控制器和行为服务器都只发布到 `/cmd_vel/nav`。Nav2 的 `smoother_server` 用于路径平滑，不是第二个速度平滑器。控制权管理器观测到扫描/TF 故障时清除缓存源命令，并拒绝保护条件异常期间收到的源命令。最终门控在观测到故障后同样清除缓存的监测器命令，因此恢复后两个节点都要分别收到新的源/监测器回调。源输入和 Humble 监测器输出均为无时间戳 Twist，其 Python 订阅回调没有发布时间。对任何一个节点而言，故障前产生但恢复后首次送达的命令，都无法与恢复后发布的命令区分。源和门控超时从回调接收时开始计时，因此软件保证针对已观测故障与缓存命令，不保证跨话题的发布顺序。实物运行前应在目标设备上验证恢复顺序与响应。

控制权管理器启动于 `disabled` 并发布零速度。操作员或任务管理器必须向 `/control/mode` 显式发布易失性的 `std_msgs/msg/String`，其 `data` 为 `nav`、`tracking`、`teleop` 或 `disabled`。命令订阅采用可靠、易失性 QoS，深度为 1。`/control/mode_state` 以 `std_msgs/msg/String` 发布已接受模式，使用可靠、瞬态本地 QoS，深度 1。初始值为 `disabled`；每个有效命令都会产生回显，即使与当前模式相同，无效命令不回显。后加入的订阅者可读取最新状态；需要每次新回显的观察者必须持续订阅。回显确认管理器处理过模式命令，但 String 消息不含请求 ID，无法识别发送者。例如：

```bash
ros2 topic pub --once /control/mode std_msgs/msg/String "{data: nav}"
```

任务导航使用 `/control/acquire_nav`（`abot_control_interfaces/srv/AcquireNav`）和 `/control/release_nav`（`abot_control_interfaces/srv/ReleaseNav`）。Acquire 只在 `disabled` 时成功，原子切换到 `nav`，返回非零随机 `lease_id` 和新模式代次。Release 仅在该租约仍持有当前 `nav` 模式时切换到 `disabled`。每个有效的外部 `/control/mode` 命令（包括重复 `nav`）都会撤销租约并递增 `/control/mode_epoch`，成功获取和释放也会递增代次。`/control/mode_epoch` 是可靠、瞬态本地、深度为 1 的 `std_msgs/msg/UInt64`，初值为零。任务发送 Nav2 目标前应检查返回代次与当前模式；后续代次表明外部控制改变时必须取消目标。这些服务、模式订阅和速度回调使用节点互斥的默认回调组。

只接受当前模式对应的命令源。源数据间隔超过 0.25 秒时输出零，但保留当前模式；遥控流丢失不会自动恢复导航。重新进入模式会清空缓存命令，因此必须收到新命令。由导航切换到跟随前，任务管理器必须取消其 Nav2 目标并确认取消。发送 `disabled` 使速度链停车，但本身不取消目标。即使仿真 `/clock` 暂停，控制权看门狗仍使用单调时间。底盘驱动也需要自己的独立命令超时。

所有运动模式（含遥控）均要求新鲜的 `/scan_filtered` 数据，`scan_timeout` 默认 0.5 秒。管理器同时检查扫描 ROS 时间戳和单调时钟经过时间；缺失、延迟、未来或中断的扫描均输出零。仿真时间暂停时，重复时间戳不能延长有效期。更旧时间戳触发保守停车，恢复需收到比最后接受扫描更新的时间戳。时钟向后重置后，应重启管理器或等待时间戳追上。管理器和最终门控还拒绝异常扫描几何、不完整角覆盖、长盲区、近距离盲区，以及声明最大量程不足以覆盖碰撞区域的扫描。有效自由空间 `+Inf` 射线计为观测；NaN、负无穷和越界值不计。`config/velocity.yaml` 的暂定要求为至少 180 束、350 度覆盖、80% 可用射线、连续盲区不超过 20 度、`range_min ≤ 0.2 m`、`range_max ≥ 1.0 m`。接受或调整这些界限前，应实测雷达、滤后扫描、外参与碰撞区域。已报告 X3 Pro 标称采样 3 kHz、每秒 4–8 圈、量程 0.10–8 m；启用任何运动模式前，其实际 `/scan_filtered` 覆盖和时序必须在目标设备上通过检查。扫描坐标系还必须能在扫描时间戳变换到 `odom`，最新 `odom -> base_footprint` 也必须通过同样的新鲜度检查。因此，即使雷达持续运行，状态估计丢失也会停止遥控。这些保护是必要的，因为 Humble 碰撞监测器可能忽略过期观测源，或在里程计 TF 消失时停止转发零速度；它们不能代替物理传感器标定和底盘独立看门狗。

footprint、碰撞区域、速度、加速度、IMU 噪声、EKF 协方差和 AMCL 运动噪声来自旧配置或 Nav2 默认值，均属暂定。真机导航前应实测调参。没有旧 PGM/YAML 被作为已验证地图安装。建图后使用 `map_saver_cli`，在定位或任务使用前验证地图及坐标。

2026-09-23 的本地依赖检查发现已安装 Nav2 1.1.20 和 slam_toolbox 2.6.10。基础 `/opt/ros/humble` 中缺少 `robot_localization`、`imu_filter_madgwick` 和 `twist_mux`，目标部署必须提供。固定版本的本地验证依赖通过 `deployment/use-validation-deps.bash` 加载。采用合成输入、真实节点的烟测已运行这些节点及 Humble Nav2 参数文件；物理传感器兼容性、标定和驾驶验收仍未完成。当前验证结果及范围见 [迁移状态](MIGRATION_STATUS.md)。
