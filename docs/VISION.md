# 视觉感知与跟随

感知节点提供观测；跟随命令经统一速度链执行。运行接口见 [接口约定](INTERFACES.md)。

<a id="perception"></a>

## 视觉检测

所有节点接收一路 `sensor_msgs/msg/Image`（默认 `/camera/image_raw`），发布带原始图像时间戳和光学坐标系的 `vision_msgs/msg/Detection2DArray`。每张已处理图像都会产生数组，目标丢失时发布空数组。像素边界框仅为二维。节点不打开相机，也不估计实际距离或位姿。

检查到空图、截断图像或不支持的编码时，发布带该图像原始消息头的空数组。相机流停止后，检测器不会把旧观测作为心跳重发；使用者必须结合采集时间和单调时钟超时判断检测失效。

| 可执行程序 | 输出话题 | 类别 ID | ROS 1 参考 |
|---|---|---|---|
| `color_detector` | `/perception/colors` | `red`, `yellow`, `green` | `abot_object_detect/node/abot_hsv_detect.py` |
| `fire_detector` | `/perception/fire` | `fire_candidate` | `color_pkg/scripts/fire_detector.py`（暂定颜色/轮廓启发式） |
| `tag_detector` | `/perception/tags` | `tag:apriltag_36h11:7` 等 | `track_tag` 标记任务 |
| `template_detector` | `/perception/templates` | `template:10` 等 | `abot_find` 数字图像资源 |
| `person_detector` | `/perception/people` | `person` | `abot_object_detect/node/abot_people_detect.py` |
| `face_detector` | `/perception/faces` | `face` | `face_pkg/scripts/face_rec.py`（仅检测） |

`config/colors.yaml` 中的颜色配置是从 ROS 1 导出的暂定 HSV 起点，`score` 表示占图像面积的比例。应针对实际 OV5640 模式和光照调参。人体检测器使用 OpenCV 全身 HOG 模型，不做人脸识别；分数为截断到规定范围的 HOG 间隔值。它可能漏检人体或误识别背景，尤其是在已报告的广角镜头下，目前没有实测准确率。

火焰候选源在独立 `/perception/fire` 话题上使用旧 ROS 1 的红/黄 HSV 范围：红色 `[0,128,46]`–`[5,255,255]`、红色回绕段 `[156,128,46]`–`[179,255,255]`、黄色 `[15,128,46]`–`[50,255,255]`。保留严格启发式门槛：轮廓数大于 10，轮廓边界框面积之和大于 40 px²。两项均可在 `config/fire.yaml` 配置且属于暂定值；面积和依赖图像分辨率。通过门槛的帧输出一个合并框，类别为 `fire_candidate`，分数为限制范围后的轮廓框面积比例，不是火焰概率。它可能漏掉单个大火焰，也可能把许多红/黄小物体、灯光、反射或暖色偏色误判为火焰。尚未使用真火或代表性负样本验证。空数组清除先前候选。节点不发布 TTS 或命令，也不打开相机。通过 `ros2 launch abot_perception fire_detector.launch.py` 启动，需另外向配置的 `image_topic` 提供 `Image` 流。

人脸检测器使用旧 OpenCV Haar 正面人脸级联，发布通用二维 `face` 框。由于 Haar 检测不提供校准置信度，结果分数设为 `1.0`。旧 `face_rec.py` 还在启动时用 `face_pkg/scripts/data/` 下单一 `jackchen` 身份的 20 张 `.pgm` 训练 Eigenfaces 识别器；本次迁移不复制这些个人图像，也不发布身份标签，因此不等价于旧身份识别功能。旧 `face_pkg/package.xml` 的代码许可仍为 `TODO`，本包不再分发其 Python 脚本或训练图像。随包级联原样复制自该 ROS1 包，保留内嵌的 Intel 开源计算机视觉库许可协议（版权年份 2000，级联署名 Rainer Lienhart）。完整条款见 `cascades/haarcascade_frontalface_default.xml`；本包代码保持 Apache-2.0。使用 `ros2 run abot_perception face_detector` 启动已安装节点，它只订阅配置的图像话题。

标签检测器使用 OpenCV 可配置的 AprilTag/ArUco 字典，默认 `DICT_APRILTAG_36h11`。类别 ID 包含字典名和解码后的数字 ID。分数 1 表示标记解码成功，并非校准置信度。旧 `track_tag` 使用 ALVAR；与现有实体 ALVAR 标记的兼容性**尚未确认**。本节点只提供二维框，不能替代已标定的 ALVAR 位姿估计。

模板检测器默认从安装于 `templates/` 的 74 张原始数字 PNG 中加载旧图像 ID `10`（该集合包含 `shoot_object/116.png`）。原 `template_id` 参数仍用于选择单张图像。若需匹配小规模集合，将 `template_ids` 设置为由逗号分隔的 1–8 个正整数 ID，例如：

```bash
ros2 run abot_perception template_detector --ros-args -p 'template_ids:="10,20"'
```

非空 `template_ids` 优先于 `template_id`；空字符串则使用 `template_id`。ID 必须唯一、采用规范十进制形式，且请求的每张 PNG 必须存在于 `template_dir`。空项或全空白项、格式错误 ID、缺失图像、重复 ID，以及超过 8 个 ID 都会使启动报错。例如 `10, ,20` 无效。

模板描述子在启动时只计算一次。每帧只执行一次 SIFT 特征提取，所有选定模板共享结果，随后每个模板分别执行特征匹配与 RANSAC 单应性检查。默认处理上限为每秒 5 帧，多模板匹配开销随所选模板数量增长（最多 8 个）；更高输入分辨率也增加特征提取开销。这限制了单帧工作量，但不保证目标计算机上的特定 CPU 占用或帧率。`score` 为几何内点比例，不是类别概率。此功能仍是基础二维匹配器，没有迁移完整的 `find_object_2d` GUI、训练流程或发射逻辑。

来自 ROS 1 `src/abot_find/shoot_object2025/` 的独立 2025 图像集安装在 `templates/shoot_object2025/`，附带 SHA-256 来源清单。86 张源 PNG 全部保留：77 张数字文件位于命名空间根目录，8 张 `MarkerData_*.png` 和隐藏 `.png` 位于 `auxiliary/`，不参与数字模板查找。这组数字图像不属于默认 74 张集合，也不改变默认 ID。使用该集合的 ID 时，通过现有 `template_dir` 参数传入其安装目录：

```bash
ros2 run abot_perception template_detector --ros-args -p template_dir:="$(ros2 pkg prefix abot_perception)/share/abot_perception/templates/shoot_object2025" -p template_id:=1
```

使用 `template_ids` 时，每个 ID 都在同一个选定目录中解析。因此，即使 ID 重叠，两组数字模板仍相互独立。辅助图像不能通过 `template_id` 或 `template_ids` 选择。源集合没有独立许可证文件，不能从原模板集推断其授权。

ROS 2 适配代码采用 Apache-2.0。原始模板图像保留 `templates/LICENSE.find-object` 中的 BSD-3-Clause 条款，该许可随图像安装。独立 `shoot_object2025` 集合在当时保留的 ROS 1 目录中没有单独许可声明。

独立话题避免一个检测器的空结果清除另一个检测器的目标。可使用 `ros2 launch abot_tracking vision_follow.launch.py` 启动选定检测器与跟随器。

<a id="tracking"></a>

## 视觉、雷达与 ROI 跟随

本包将旧 `tracker_pkg` 跟随器的仅偏航部分，以及 `track_tag` 转向行为迁移到 ROS 2 速度接口约定。跟随器订阅**一个显式选定的**检测话题和 `/camera/camera_info`，要求类别 ID 符合配置，光学坐标系匹配、采集时间戳完全一致，并以 20 Hz 向 `/cmd_vel/tracking` 发布 `geometry_msgs/msg/Twist`。它使目标朝图像中心对齐，线速度为零。目标缺失、边界框异常、检测过期/重放或 CameraInfo 过期都会产生新的零命令。节点使用 CameraInfo 的宽度，不假设 640 像素模式，也不根据未经测量的相机对角视场推算方位。

启动时选择检测源与目标：

```bash
ros2 launch abot_tracking vision_follow.launch.py source:=color color:=red
ros2 launch abot_tracking vision_follow.launch.py source:=person
ros2 launch abot_tracking vision_follow.launch.py source:=template template_id:=10
ros2 launch abot_tracking vision_follow.launch.py source:=template \
  template_ids:=10,20 template_id:=20
ros2 launch abot_tracking vision_follow.launch.py source:=tag tag_id:=7
# 若实体标记使用其他受支持的 OpenCV 字典，可选：
ros2 launch abot_tracking vision_follow.launch.py source:=tag \
  tag_dictionary:=DICT_5X5_250 tag_id:=7
```

上述标签示例假设使用 `36h11` AprilTag，使用前应确认实体标签族及 ID。模板 ID `10` 是安装样本，不表示它就是实际目标。启动参数包括 `image_topic`、`camera_info_topic`、`template_dir`、`template_ids`、`use_sim_time`、`enable_detector` 和 `enable_follower`。按检测源区分的初始超时为：颜色/标签 0.3 s、模板 0.5 s、3 Hz HOG 人体检测器 0.8 s，可用 `detection_timeout` 和 `camera_info_timeout` 覆盖。这些值限制因目标过期而持续转向的时间，需要实测相机频率。对 `source:=template`，非空 `template_ids` 最多选择 8 个检测资源；`template_id` 仍指定跟随器跟随的**唯一**类别，启用跟随器时必须位于列表中。空 `template_ids` 保留单模板行为。`color_follow.launch.py` 仍是仅颜色入口。

这些启动文件不启动相机、底盘驱动或速度链，也不自行授予 `tracking` 模式。已有控制权管理器、平滑器、碰撞监测器与最终门控必须运行，并收到新鲜雷达及里程计 TF。管理器选择 `/control/mode = tracking` 前必须取消 Nav2 目标并确认取消。选择 `disabled` 会撤销运动控制权。同一时间只运行**一个** `vision_follower`；多个 `/cmd_vel/tracking` 发布者会竞争，ROS 图发现不能充当可靠锁。跟随器不能取消 Nav2 目标或触发旧 `/shoot` 命令。增益、检测阈值和目标稳定性需现场调参；实物阶段门槛通过前，真实目标跟随仍未验收。

### 暂定雷达跟随器

`lidar_follow.launch.py` 只迁移旧 `lidar_follower/scripts/laserTracker.py` 有依据的行为：在前方扇区选择最近有限回波，且前一帧扫描中相邻两束范围内有相近读数。旧 `follower.py` 以 `active=False` 启动，手柄订阅被注释；其 `/object_tracker/target_position` Point 输入还与跟踪器的 `/object_tracker/current_position` 自定义消息输出不匹配。因此，ROS1 快照不能证明有可运行的雷达跟随链、人体检测器或持久目标锁定，当前 ROS2 节点也不声称具备这些能力。

```bash
ros2 launch abot_tracking lidar_follow.launch.py
# 仅在静止台架检查和显式控制权选择后执行：
ros2 launch abot_tracking lidar_follow.launch.py enable_linear_motion:=true
```

节点以兼容尽力而为 QoS 的方式订阅 `laser_link` 坐标系下的 `/scan_filtered` LaserScan，以 20 Hz 在 `/cmd_vel/tracking` 发布可靠 Twist，同时使用扫描消息头时间和单调接收时间。首帧、回波缺失或无效、异常/局部扫描、坐标系或几何变化、重复/乱序时间戳、未来/过期时间戳、间隔超过 0.3 s 或目标年龄超过 0.4 s，均输出零。仿真时钟回退清除两帧历史。扫描须至少覆盖 350°、包含前方 ±45°、至少 180 束，并具有足够可用角覆盖。选定点距离须为 0.35–2.5 m，与前次读数差距不超过 0.15 m。这些是暂定门槛，不是实测检测性能。

默认线速度始终为零。显式覆盖启动参数后，跟随器对目标距离 1.0 m 使用比例误差，速度限制在 ±0.10 m/s，且只有目标位于正前方 0.15 rad 内才平移。偏航速度限制在 ±0.25 rad/s。旧 ROS1 PID 的积分和微分项未迁移，因为旧控制器没有可运行的激活路径，也没有目标过期看门狗。前向扇区和对齐门槛也不同于旧全角度最近回波搜索。

启动文件不设置 `/control/mode`。管理器必须取消并确认所有 Nav2 目标，再在已有控制权节点中显式选择 `tracking`。运动前必须具备平滑器、碰撞监测器、最终门控、新鲜扫描和里程计 TF。`/cmd_vel/tracking` 只能有一个发布者，不能同时运行 `vision_follower` 与 `lidar_follower`。由于算法没有目标身份，更近障碍物可能替换目标。启用直线运动前，实物 X3 Pro 扫描时序、雷达轴向、目标行为、停车距离和碰撞控制链仍需静止台架与实车验证。

### 带种子 ROI 的 LK 观测

`roi_lk_tracker` 迁移 ROS 1 `tracker_pkg/scripts/lk_tracker.py` 的特征跟踪行为，不包含其 GUI 选择、假设相机视场、角度发布、手柄处理或直接速度输出。节点从 `/camera/image_raw` 接收 `sensor_msgs/msg/Image`，从私有话题 `/roi_lk_tracker/roi_seed` 接收 `vision_msgs/msg/Detection2DArray` 种子。种子数组必须**恰好包含一个**检测，数组头与检测头的非零采集时间戳及光学坐标系一致。唯一假设必须为 `class_id: roi_seed` 且分数为有限正数。轴对齐边界框使用像素中心 `(x,y)`、`size_x`、`size_y` 和 `center.theta: 0`，须完全位于对应时间戳图像内。种子可以略早或略晚于图像到达；不匹配、过期、重复、乱序、旋转、异常或超出图像的种子都不能启动跟踪。节点最多保留四张图像（每张不超过 2,073,600 像素，即 1920×1080）和一个待处理种子。

私有输出 `/roi_lk_tracker/roi` 同样为 `vision_msgs/msg/Detection2DArray`。有效观测携带**源图像的时间戳和坐标系**、一个 `class_id: roi` 检测及当前像素框。新图像上的空数组表示目标丢失。若种子先到，初始种子图像即可产生观测；若种子后到，首个观测使用下一张图像。不把缓存 ROI 作为心跳重发。图像头可信且新鲜时，解码失败立即发布空数组；无效种子中断活动跟踪时，在下一张有效图像发布空数组。相机停止后，下游必须使最后观测超时失效。

图像和种子的年龄同时按 ROS 采集时间与单调接收时间限制为 0.3 s，最大图像间隔为 0.25 s。节点拒绝超出 1 µs 舍入容差的未来时间戳，也拒绝重复或乱序时间戳。坐标系/像素几何变化、时钟不连续、特征缺失、光流不一致及 ROI 离开图像都会清除跟踪。Lucas–Kanade 最多使用 200 个 Harris 角点，要求前后向一致性在 1 px 内、中值光流一致性在 3 px 内，且至少剩余 8 个点。按中值光流平移原尺寸框。旧 ROS 1 代码使用剩余特征点的包络作为边界框，可能收缩到某个纹理簇。这些阈值均为暂定，需针对相机实测。

```bash
# 仅观测，不启动 /cmd_vel/tracking 发布者。
ros2 launch abot_tracking roi_track.launch.py
# 静止检查完成后，显式加入现有仅偏航跟随器。
ros2 launch abot_tracking roi_track.launch.py enable_follower:=true
```

可选跟随器要求 `/camera/camera_info` 上的 `CameraInfo` 与图像**时间戳和坐标系完全一致**，沿用 `roi` 类别选择与新鲜度门槛。它是此启动文件唯一的 `/cmd_vel/tracking` 发布者，不要在该话题启动其他视觉或雷达跟随器。启动文件不设置 `/control/mode`；在管理器取消 Nav2、完成上述物理安全检查并显式选择 `tracking` 前，控制权必须保持 `disabled`。ROI 不推断目标身份、尺度、深度、遮挡恢复或实际方位角。

<a id="templates"></a>

## 模板资源与来源

独立的 `shoot_object2025/` 目录逐字节保留了 `src/abot_find/shoot_object2025/` 顶层的全部 86 张 PNG。其中 77 个文件名为规范正十进制数的文件位于命名空间根目录，可供检测器选择。另 9 个文件（`MarkerData_1.png`–`MarkerData_8.png` 及隐藏文件 `.png`）安装在 `shoot_object2025/auxiliary/`，不参与数字模板查找。`MANIFEST.json` 记录每个源文件的安装路径、字节数和 SHA-256 摘要。将 `shoot_object2025/` 本身设为 `template_dir` 即可选择这组数字模板；ID 只在选定目录内解析，因此两组中的相同数字 ID 不会合并。源目录没有独立许可证，不能据根模板集的许可证推断其授权状态。

两组图像的内容和 ID 含义都需结合实际任务确认；其中没有图像被训练或分类为人体。

## 尚未恢复的旧功能

- 旧 ALVAR 标签与当前字典的实体兼容性、模板 ID、人物和火焰候选效果均需实测。火焰候选不自动播报或控制执行器。
- 已提供显式种子的 LK 跟踪；KCF/Kalman 连续跟踪、GUI 框选、遮挡恢复和人脸身份训练未实现。
- 旧巡线脚本包含写死中心、未初始化手柄输入和直接发布底盘速度等缺陷，未原样迁移。当前目标居中跟随不等于巡线；若需要巡线，应先确定胶带与场景并验证纯视觉偏移。
- OpenCV 教程封装、重复图像桥接和错误标注坐标系的旧位姿转发不作为独立功能保留。比赛专属标签对齐与附件触发顺序仍待地图和任务策略确认。
