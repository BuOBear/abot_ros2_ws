# 验证与实车验收

下表是已有记录的汇总，本次文档整理未重新运行 ROS 构建或设备测试。所有证据路径相对于项目根目录。

## 已有证据与适用范围

| 记录 | 结果与范围 | 证据 |
|---|---|---|
| 合并并更名后的工作区 | 15 包构建，247 tests，0 errors / failures / skipped | `.validation/restructure-verify.log` |
| 默认机械臂组合启动 | 仅 `robot_state_publisher`，硬件和机械臂关闭 | `.validation/manipulator-default/result.json` |
| 更名后组合建图（域 154） | 合成输入形成 map→odom→base_footprint，9 个生命周期节点 active，唯一最终速度发布者，3 个管理器有序关闭，launch 返回 0 | `.validation/restructure-mapping-154/result.json` |
| 合并时启动前检查 | 未标定机械臂、缺失 YOLO 权重在创建子进程前拒绝；记录含更名前路径 | `.validation/merge-gated-result.json` |
| 历史严格 P0（域 145） | 通过；最大输入间隔 0.107 s，扫描过期后首次零输出 0.453 s | `.validation/p0-smoke-cyclone-145/results.json` |
| 历史严格 P0（域 146） | 通过；最大输入间隔 0.105 s，扫描过期后首次零输出 0.448 s | `.validation/p0-smoke-cyclone-146/results.json` |

2026-09-26 的 P0 基线为 13 包、202 项测试（`.validation/p0-acceptance-verify-13pkg.log`）。严格双次运行覆盖合成 SLAM 建图/保存/重载、AMCL、实际 Nav2 非零速度传递、精确目标最终 CANCELED、人工接管、无效/过期扫描及过期里程计 TF 后持续六分量精确零，并确认四个生命周期管理器有序关闭，无 bond 缺失、SIGKILL 升级、强杀或所属进程组残留。它不证明实车到点或比赛任务成功。

当时烟测脚本 SHA-256 为 `2d7e781512258bce7ee1ee1d822265ba3bc9fb83495fc6bf7c62842eaf98991c`，Cyclone XML 为 `acd5ee8c0bcb5d179d6c29b13943158f98cd9c15acb88f6ef3c420ccd5a8d435`。合并/更名后的验证未重复完整 P0 双次运行，不能把不同阶段记录拼成一次新的严格验收。

## 离线复现

普通目标主机先按 [部署说明](DEPLOYMENT.md) 安装依赖，再执行 `bash tools/verify.sh`。本 WSL 主机使用同文所述的本地依赖与 Cyclone 叠加层。以下命令在项目根目录、新 shell 中执行：

```bash
source deployment/use-validation-deps.bash
source .validation/cyclone-deps/install-rmw/local_setup.bash
export LD_LIBRARY_PATH="$PWD/.validation/cyclone-deps/install/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_PREFIX_PATH="$PWD/.validation/cyclone-deps/install:${CMAKE_PREFIX_PATH:-}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1 PYTHONUNBUFFERED=1
export CYCLONEDDS_URI="file://$PWD/deployment/cyclonedds-local-test.xml"
unset FASTRTPS_DEFAULT_PROFILES_FILE
# 选择未使用的隔离域，输出目录也应使用新的名称。
export ROS_DOMAIN_ID=155
bash tools/verify.sh
source install/setup.bash
python3 tools/smoke_p0.py --domain 156 --bond-timeout 12 \
  --output .validation/p0-rerun-156
python3 tools/smoke_p0.py --domain 157 --bond-timeout 12 \
  --output .validation/p0-rerun-157
export ROS_DOMAIN_ID=158
python3 tools/smoke_manipulator.py --mode mapping --timeout 45 \
  --output .validation/manipulator-rerun-158
```

已有通过环境为 WSL2 Ubuntu 22.04、4 CPU、约 3.8 GiB 内存及 2 GiB 交换空间，Cyclone DDS 0.10.5 / RMW 1.3.5。XML 使用参与者索引上限 64、消息/分片 1200/1025 字节；12 s bond 仅用于 WSL 烟测，生产默认 4 s。早期 Fast DDS 有消息传输、输入停顿及关闭问题，Cyclone 默认配置也曾出现参与者耗尽和大消息丢失；早期失败/中止不计入验收。确切 WSL 丢包机制未确定，不把此回环配置用于生产多机网络。

可选离线检查脚本包括 `tools/smoke_vision.py`、`tools/smoke_visual_sources.py` 和 `tools/smoke_roi_multi.py`，分别覆盖合成视觉跟随、多检测源及多模板/ROI 图连接。真实相机、推理模型、音频、板卡与远程服务仍需单独验证。

## 重新验收的判定规则

1. 当前 15 个项目包全部构建，测试错误、失败和跳过为零，记录实际测试数、环境和日志。旧 ROS1 哈希清单只作历史记录，不要求恢复已删除源码。
2. 检查安装态启动参数、默认仅描述启动和 TF 单发布者；建图/定位模式需有预期 map→odom→base_footprint→base_link 链。
3. 协议及伪终端测试覆盖无效帧、断开/重连、看门狗、生命周期和有界关闭。
4. 完整 P0 运行覆盖地图保存/重载、AMCL、Nav2 命令传播、精确目标取消、人工接管、扫描/TF 故障后连续精确零；恢复需清缓存且等候新回调。无时间戳 Twist 不提供原始发布时间保证。
5. 全程输入间隔符合新鲜度限制，生命周期管理器均成功关闭，无 bond 丢失、强杀或残留。不得以放宽停车/清理标准取得通过。
6. 同一已记录环境中，在全新域连续通过两次完整严格测试，并保存两组结果和进程日志；组合建图或测试数量不能替代此项。

## 实车台架清单

当前实现按麦克纳姆底盘保留 `vx/vy/wz`。本文件用于接入实车后的记录；
所有空项均表示未测量，不应用离线模拟结果替代。

| 基线项目 | 实车记录 |
|---|---|
| 当前 ROS1 主 launch、必需任务入口 | 历史硬件入口候选 `abot_bringup/robot.launch`，导航入口为 `robot_slam/navigation.launch`；实际运行命令待核实 |
| 上位机型号、CPU 架构、系统版本 | 用户确认 Ubuntu 22.04 x86 mini 主机；具体 CPU/内存/USB 布局待现场记录 |
| MCU 与固件版本、烧录基线 | 用户确认 STM32F103VET6，固件已烧录在下位机，源码可能遗失；项目中有上位机串口协议和驱动源码。现场读取板上版本与记录当前烧录基线，不要求以找回源码作为迁移前提 |
| 底盘通信 | 用户确认 USB 转 TTL；旧 YAML 候选 `/dev/abot`、921600，适配器设备名、USB ID、固件串口设置待核实 |
| 轮序、转向符号、滚子布局、轮径、纵横轮距 | 待测量 |
| 雷达型号、波特率、唯一设备标识 | 用户确认 YDLIDAR X3 Pro；官方型号表为 115200、3 kHz、0.10–8 m、4–8 Hz (PWM)；实际串口、DTR/电机接线、转速和扫描质量待核实 |
| 相机型号、采集格式/分辨率、标定文件 | 用户确认 USB 免驱 OV5640，200 万像素、对角 148°、最大 120 fps，并指定沿用旧 640×480 标定。ROS2 已安装 `track_tag/camera_calibration.yaml` 的副本并作为默认 `camera_info_url`；实际 V4L2 模式/帧率、图像与 CameraInfo 对应关系仍待设备侧核对 |
| base/IMU/laser/camera 外参 | 待实测；xacro 当前继承旧模型候选值 |
| 实际 footprint，包括新增外设 | 待测量 |
| ROS1 bag、地图、频率与任务成功率 | 用户确认三场比赛均在 2025 年，但记不清地图和脚本对应关系。ROS2 `abot_maps` 仅保存 `1` 和 `shoot` 两组 PGM/YAML、三张参考示意图；三套比赛记录仍未分配。当前源树未见实车 bag、频率或任务成功率记录 |

准备工作和生命周期命令见 [部署说明](DEPLOYMENT.md) 与
[底盘说明](BASE.md#hardware)。控制源先保持 `disabled`，显式选择模式后
才测试速度。驱动不写固件参数、不清零 MCU 里程计，也不发 odom TF。
在 mini 主机上先运行 `bash tools/inventory_target.sh > target-inventory.txt`
采集只读 USB、串口和 V4L2 模式清单，再填写上表缺项。

| 验收项目 | 记录方法 / 必须观察的结果 | 状态 |
|---|---|---|
| 串口与固件 | 分辨底盘与雷达两个 USB 串口并建立稳定命名；底盘单一串口持有者，记录版本响应、查询往返时间、错误帧与重连 | 待验 |
| 速度与里程计 | 前后、纯横移、斜移、原地旋转，逐项确认 vx/vy/wz 的方向、比例和时间戳 | 待验 |
| 独立停车 | 用户确认主机断电或串口断开后 MCU 会立即控制电机停止；静止台架测量实际停止时限并记录断连、断电路径，主机发出零帧不等于车轮已停止 | 待验 |
| 重连 / 停用 | 停用后归零，断线期间无旧反馈重发，恢复后不重放断线前命令 | 待验 |
| IMU | 静止/转动时确认 m/s²、rad/s、轴方向、重力符号，测量 bias 与噪声 | 待验 |
| TF 与融合 | 每个子 frame 唯一父节点及发布者；EKF 独占 odom→base_footprint；横移速度真实可用 | 待验 |
| 激光与相机 | 验证 X3 Pro 115200 通信、转速、DTR、电机启停、完整 360° 扫描、有效束比例、QoS、时间戳和滤波边界；枚举 OV5640 V4L2 模式，核对图像/CameraInfo 帧和广角标定 | 待验 |
| 速度链 | 侧面和后方障碍、扫描过期、模式切换和遥控失联均产生预期停车行为 | 待验 |
| 导航控制租约 | 任务取得 Nav 租约后才发 goal；人工切到 teleop/tracking 时任务取消该 goal；旧租约释放请求不能覆盖人工模式或后来重选的 nav | 待验 |
| 视觉与任务 | 用实物验证颜色、标签制式/ID、模板和人物检测；确认多模板列表、单一跟随目标及目标主机上的 CPU/帧率；核对图像与 CameraInfo 时间戳、地图身份、实测航点、到点后观察超时与目标丢失 | 待验 |
| ROI LK 跟踪 | 用真实相机画面与操作端生成同帧同光学 frame 的 `roi_seed`；测量纹理不足、遮挡、画面切换、分辨率变化和断流后的观测/跟随停车；保持跟随关闭直至静止台架检查通过 | 待验 |
| P2 感知与跟随 | 用代表性正负样本测人脸框与火焰颜色候选的误报/漏报；核对 X3 Pro 扫描时序、雷达轴、最近目标切换、停机距离和跟随源互斥；线速度保持关闭直至静止台架检查通过 | 待验 |
| 独立外设 | 区分 `/dev/shoot` 与底盘/雷达适配器；逐帧核对 8 字节协议、stop 效果、断连和重复请求语义；仅在静止台架上显式调用 | 待验 |
| 语音与 VLM | 验证 WAV 采集交接、目标主机上的模型耗时/识别效果；可选播报需单独安装 eSpeak NG 并核对中文音质与播放时限；VLM 用授权凭据验证真实网络调用、超时、取消和晚结果隔离 | 待验 |

以上记录通过后，总控再开放 M3 实车建图/定位/导航验收。M3 仍需真实场地的
地图保存与重载、单/多目标导航、取消/恢复，以及计划中的重复成功率验证。
用户已选定颜色、标签、模板物体和人物的视觉识别与跟随。M4 已有受限离线实现；现场仍需确认标签制式与 ID、模板对应实物、人物检测效果、相机标定和
实际任务流程。上述台架、定位和导航门槛未通过前，不进行实车业务验收。

## 机械臂验收门槛

按 [机械臂与腕部视觉](MOBILE_MANIPULATOR.md) 逐项核对供电/信号、板卡固件、上电动作、实际通道和机械限位，再配置标定门控。分别验证真实相机采集、YOLO11 样本推理与类别、腕部内外参、可信姿态、碰撞边界、底盘停止互锁、执行失败和抓取结果。串口写入成功不等于到位；没有这些证据前，不作自动抓取验收结论。
