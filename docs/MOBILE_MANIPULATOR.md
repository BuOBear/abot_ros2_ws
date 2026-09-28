# ABOT 机械臂与腕部视觉

这是从仿真工程拆出的真机接入层，复用 `/home/manbo_u22/abot_ros2_ws` 已有的 ROS2 底盘实现。旧 ROS1 仿真源码已删除。**当前完成基础接入代码和离线验证，尚未完成实物标定、真机验收及比赛自动抓取恢复。**

## 已确认的硬件

- 原 ABOT 与机械臂工程使用同一台麦克纳姆全向小车：STM32 底盘、YDLIDAR X3 Pro、普通 USB 摄像头。
- 摄像头移到了机械臂上。用户指出外观位置在尺寸图从上往下数第二个关节左侧、夹爪左下方；这不是已标定的安装变换，不能再用原先底盘到相机的固定 TF。
- 5 个运动关节 + 1 个夹爪舵机。图片中的编号 6 是夹爪，不是第六个旋转自由度。
- 六个舵机均插在 24 路控制板输出，上位机经控制板串口发命令；不是仿真中的 RM65。具体插入通道仍需核对。
- 真实比赛使用 YOLO11；用户确认原比赛权重已丢失。通用权重只能验证推理链路，不能替代比赛识别能力。

## 已实现与边界

| 部分 | 本工作区行为 | 仍需实物验证/补充 |
|---|---|---|
| 底盘/导航 | 复用 abot_hardware、robot_localization、Nav2、SLAM Toolbox 和 abot 的最终速度控制链，保留横移 | abot 原有底盘标定；加装手臂后的 footprint、载荷、限速和导航姿态 |
| 激光/相机 | 复用 abot 的 YDLIDAR 与 usb_cam；相机 frame 改为 wrist_camera_optical_frame | USB 设备名、实际格式、当前分辨率的内参 |
| 舵机板 | 按提供的板卡资料实现 9600/8N1、A–X 通道、固定三位角度；逐轴 ROS2 服务 | 当前实物板固件/电气接口、实际通道和各轴可用角度范围 |
| 检测 | Ultralytics YOLO11 接入，输出标准 vision_msgs/Detection2DArray，保留类别/置信度/原图时间戳 | ultralytics 运行环境、YOLO11 权重及真实图像推理验证 |
| 单目目标估计 | 可选的已知平面交点节点，使用相机内参和拍摄时刻 TF | 平面方程、真实腕部姿态/手眼标定；默认不启动 |
| 自动抓取 | 尚未启用 | 机械臂尺寸、轴向、零位、板角度与关节角映射、夹爪/TCP、碰撞模型、可验证的执行机制 |

单目没有深度图。平面交点仅表示检测框中心射线与指定平面的交点；有高度的物体会产生偏差，不能当成物体真实三维中心或完整抓取位姿。五自由度手臂也不能保证任意六维末端位姿可达。

没有复制旧代码中的固定 0.45 m 深度、固定六关节动作、全类别强制改成 apple、Gazebo 里程计或静态 map→odom。没有发布估计/指令角度为实测 `/joint_states`，因此未标定的腕部 TF 不存在，三维估计会拒绝输出。

## 构建

前置：Ubuntu 22.04 + ROS2 Humble，ROS Python、vision_msgs、cv_bridge、xacro、pyserial、NumPy、OpenCV、PyYAML 可用。底盘运行依赖沿用 abot 的部署说明。

```bash
source /opt/ros/humble/setup.bash
cd /home/manbo_u22/abot_ros2_ws
colcon build --symlink-install
source /home/manbo_u22/abot_ros2_ws/install/setup.bash
ros2 run jy_real_robot preflight
```

不要在此 shell 中 source 旧 catkin 的 devel/setup.bash，也不要同时启动 abot 的整套 bringup；本工作区已经组合了同一套底盘节点。

机械臂模块包含两个包（统一工作区共 15 个包）：`jy_real_interfaces`（串口命令结果服务）与 `jy_real_robot`（组合启动、舵机驱动、检测、平面估计、预检）。标准相机/检测/TF/导航使用 ROS2 现成接口；板卡没有标准硬件驱动，单独实现协议适配。

## 分阶段启动

默认启动仅发布底盘模型，不打开串口、相机、雷达，不发运动命令：

```bash
ros2 launch jy_real_robot real.launch.py
```

先验证 USB 相机图像：

```bash
ros2 launch jy_real_robot real.launch.py enable_camera:=true
```

`camera_config` 默认指向安装包中的 `config/camera.yaml`。设备默认 `/dev/video0`，格式沿用 abot 的候选配置，分辨率 640×480。`camera_info_url` 默认空，不冒用旧标定；确认当前镜头、分辨率后传入 `file:///绝对路径/标定.yaml`。仅二维推理不依赖内参，三维估计必须有有效 CameraInfo。

YOLO11 需要在启动节点的同一 Python 环境安装 Ultralytics，并准备本地 `.pt` 检测模型。参考 [YOLO11 官方说明](https://docs.ultralytics.com/models/yolo11/) 和 [预测结果接口](https://docs.ultralytics.com/modes/predict/)。本次没有安装 Torch/Ultralytics，也没有下载模型。

```bash
ros2 launch jy_real_robot real.launch.py enable_camera:=true enable_yolo:=true \
  weights:=/绝对路径/你的YOLO11检测权重.pt inference_device:=cpu
```

空路径或不存在的权重会启动失败，不自动下载、不切换到白色物体检测。模型必须是 Ultralytics 可加载的 detection 权重；文件名不能证明训练类别或架构，部署时还需核对模型元数据和真实样本结果。通用 YOLO11 权重只能用来验证识别链路，不能证明恢复了比赛模型。

恢复底盘/雷达/滤波/速度控制组件可设置：

```bash
ros2 launch jy_real_robot real.launch.py \
  enable_hardware:=true enable_lidar:=true \
  enable_state_estimation:=true enable_velocity:=true
```

底盘继续使用 abot 的生命周期节点；不会由此自动配置或激活，按 abot 的真机调试流程处理。通过相应验收后，建图加 `mode:=mapping`，导航加 `mode:=localization map:=/绝对路径/真机地图.yaml`。不沿用仿真场地的固定导航点。新节点不直接向最终底盘速度话题发命令。

## 舵机板接线与标定

通道可以任选，但实际接线必须与配置一致。默认约定：

| 机械臂执行器 | 控制板通道 | 协议字母 |
|---|---|---|
| joint_1（图片 1） | 1 | A |
| joint_2（图片 2） | 2 | B |
| joint_3（图片 3） | 3 | C |
| joint_4（图片 4） | 4 | D |
| joint_5（图片 5） | 5 | E |
| gripper（图片 6） | 6 | F |

复制 `src/jy_real_robot/config/arm_profile.yaml` 为本机配置，填入各执行器实测的 `min_deg/max_deg`，确认接线后再设 `calibration_verified: true`。角度是控制板的 0–180 指令单位，不是 URDF 弧度或舵机外部实际旋转角。板卡资料中的 270° 舵机需要不同 PWM 换算；用户另提供的 `舵机.jpg` 将机身五个舵机标为约 300° 行程，也不能直接换算成此命令范围。当前实现不假定装有修改版固件。

`/mnt/c/Users/puppet/Downloads/舵机.jpg` 描述的是舵机本体：机身舵机标称 6.0–7.4 V、约 300°±15°，夹爪舵机标称 4.8–6.0 V、约 180°±10°；图片还写有 115200 UART 和位置/状态回读。用户已确认实际六个舵机插在 24 路板输出。**115200 和回读说明不能套用到当前 9600 板卡链路**；在接电或发送命令前，须核对本体、板卡输出的电压、信号类型、针脚顺序及固件兼容性。当前 ROS 服务不能读到该图片所述的舵机内部状态。

```bash
ros2 run jy_real_robot preflight \
  --arm-profile /绝对路径/本机arm_profile.yaml --arm-port /dev/jy_arm
ros2 launch jy_real_robot real.launch.py enable_arm:=true \
  arm_profile:=/绝对路径/本机arm_profile.yaml arm_port:=/dev/jy_arm
```

`/dev/jy_arm` 是部署时设置的稳定设备别名，本次没有添加 udev 规则，也没有猜 USB VID/PID。可显式传真实串口设备路径。资料描述的控制入口是板上 USART3 的 TX/RX，不能仅凭 Type-C 插口推断其为同一通信入口。

命令接口 `/arm/set_servo_angle`：类型 `jy_real_interfaces/srv/SetServoAngle`，请求包含 `actuator` 和整数 `board_angle_deg`。只有接线、机械限位、各姿态碰撞检查完毕后才发送测试角度。启动不回零，不发送默认姿态，不自动重连或补发。相邻命令小于 0.2 秒会被拒绝；串口短写或异常会锁定故障，必须检查后重启。

响应 `sent_unverified=true` **仅表示六字节已写入主机串口**，不代表板卡已收到、舵机已到位或夹住物体。协议没有本实现可以验证的到位、同步轨迹、速度、急停或扭矩关闭命令；关闭 ROS 节点也不代表舵机停止保持当前指令。驱动不伪装为 FollowJointTrajectory 闭环控制器。

厂家《24路舵机控制板.pdf》的初始化示例把所有通道设为 90°。这是板卡固件行为，主机“不发启动动作”不能消除板卡上电动作；实物调试前应核对当前固件。不要直接运行资料中的往返扫角示例作为装好机械臂后的测试。

## 单目抓取的后续数据

`table_target` 是可选节点，默认不启动。它要求 `calibration_verified=true`、`target_class`、`plane_normal`、`plane_offset` 和有效的腕部相机 TF；按检测原图的时间戳、光学帧和图像尺寸匹配 `CameraInfo`，并按该采集时刻查询 TF。旧数据、无标定、无 TF、平行/反向/超量程射线会被拒绝。输出 `/perception/plane_point`（PointStamped），没有运动输出。

当前模型仅包含真实底盘。要恢复自动抓取，需要实测以下内容，不能从 RM65 模型或照片推算：

1. 五个关节的轴向、连杆长度、安装位姿、舵机零位与方向、夹爪开合范围和 TCP。
2. 腕部相机内参、手眼外参，以及每张图像时刻可信的腕部姿态。开环指令位置不是实测姿态；固定观察位的独立标定可作为受限方案，移动后必须失效。
3. 比赛类别和权重，或重新采集真实数据训练；明确所需抓取姿态及场地地图。
4. 机械臂执行/失败判断、抓取验证、底盘停止与机械臂动作互锁，再接入 MoveIt 2 或适配五自由度的受约束抓取策略。

## 验证与剩余模型数据

统一工作区的构建、默认启动、组合建图及启动前拒绝检查统一记录在 [验证与验收](VALIDATION.md)。尚未完成真实串口板、舵机、相机、YOLO11 推理、动态腕部 TF 和自动抓取验收。

现有尺寸资料只有约 300 mm 工作半径、135×260 mm 底座、472.55 mm 总高及 82.85、79.05、107.50 mm 局部尺寸。缺少轴线、尺寸基准、安装变换、关节限位和碰撞几何，不能据此生成可信机械臂模型。

自动操作还缺任务 Action、底盘与机械臂互锁、执行失败与抓取结果反馈。`/wheel_odom` 没有设备时间戳/序号，也不与新运动命令原子互锁；接近零的反馈速度不能单独证明底盘已安全停止。

## 协议依据

读取资料目录 `/mnt/d/share/24路舵机控制板`，没有执行其中的控制程序：

- `3.程序源码/上位机控制源码/树莓派-控制舵机旋转/serial_pi_cor.py`：9600/8N1、A–X、0–180。
- `3.程序源码/上位机控制源码/C51-控制舵机旋转/24CServo-uart/main.c`：`$` + 通道字母 + 三位十进制 + `#`；1 号通道对应 A。
- `2.上位机和控制板进行控制/1.树莓派、jetson控制舵机控制板/上位机控制.pdf`：USART3，上位机控制 D 通道示例。
- `1.舵机控制板介绍和程序分析/24路舵机控制板.pdf`：上电角度示例、180/270 换算差别。资料中的字符回显测试是独立测试程序，不能当成工作固件的动作 ACK。

厂家 Python 对 1–9° 少补一个零，本实现按 C51 的固定三位算法编码，例如 `$A001#`；不照搬该格式错误。
