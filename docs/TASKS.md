# 地图、任务与扩展模块

所有任务和外设均需显式启用；赛事登记不自动生成可执行路线。

<a id="maps"></a>

## 地图与赛事

用户确认本 ABOT 项目参加过 2025 年三场不同比赛，但无法回忆每场所用地图和脚本。旧 ROS 1 源码树随后按用户要求删除。下列启动与脚本关联保留自源码盘点的历史发现，不表示源码仍存在，也不表示赛事对应关系已确定。

### 保留地图与参考图

ROS 2 仅保留两组导航地图：

| 场地记录 | 保留地图 YAML 与图像 | 用户提供示意图 | 证据 |
| --- | --- | --- | --- |
| 3x3 `shoot` | `abot_maps/maps/robot_slam/shoot.yaml` 与 `shoot.pgm` | [目标及起终点](../src/abot_maps/layouts/field_3x3_targets.png) | 用户确认地图与示意图配对 |
| 3x3 `1` | `abot_maps/maps/robot_slam/1.yaml` 与 `1.pgm` | [墙体与起点](../src/abot_maps/layouts/field_3x3_walls_start.png) | 用户确认这是另一张 3×3 地图，示意图配对由排除法推断 |
| 3x1.5 | 未保留导航地图 | [目标及起终点](../src/abot_maps/layouts/field_3x1_5_targets.png) | 只有示意图，不计划恢复地图 |

`abot_maps/layouts/` 中三张图属于参考文档，不是占据地图或运行时地图选项。地图图像字节和 YAML 配置保留，图像路径改为相对形式。`abot_maps/maps/manifest.json` 记录历史源路径和哈希；这些路径仅说明来源，不指当前 ROS 1 源码树。选定两组地图后，旧 ROS 1 源码已按要求移除。

保留地图的尺寸和示意图标签不能证明比例尺、原点、定位质量或实测航点。两张地图均未分配给具名赛事，也都不是默认任务路线。

### 三条赛事记录

| 赛事记录 | 年份 | 已确认地图 | 已确认任务入口 | 状态 |
| --- | --- | --- | --- | --- |
| 2025 赛事 1 | 2025 | 未知 | 未知 | 未启用路线或执行器策略 |
| 2025 赛事 2 | 2025 | 未知 | 未知 | 未启用路线或执行器策略 |
| 2025 赛事 3 | 2025 | 未知 | 未知 | 未启用路线或执行器策略 |

ROS 2 `abot_mission/config/competition_registry.yaml` 将三条记录分开保留且不分配，不会启动路线。

### ROS 1 盘点记录中的历史关联

这些文件名级关联在删除源码前已发现，只表明旧启动文件引用了什么，不能确定实际用于哪场比赛。

| 旧入口 | 已记录文件级关联 | 仍未知的内容 |
| --- | --- | --- |
| `robot_slam/launch/navigation.launch` | 默认 `robot_slam/maps/1.yaml` | 哪场 2025 比赛使用，以及同时运行的任务 |
| `robot_slam/launch/navigation_shoot.launch` | 默认 `robot_slam/maps/shoot.yaml` | 对应赛事和任务 |
| `robot_slam/launch/nav_cartographer.launch` | 默认 `robot_slam/maps/my_map.yaml` | 属于比赛运行还是建图/导航试验 |
| `robot_slam/launch/2025.launch` | 运行 `navigation_multi_goals_2025.py` | 地图、赛事及所用任务分支 |
| `robot_slam/launch/multi_goal_shoot_2025.launch` | 运行 `shoot1.py`，启动文件内嵌六个候选位姿 | 地图、赛事、目标 ID 及位姿是否匹配场地 |
| `robot_slam/launch/4x5_demo.launch` | 运行 `4x5_demo.py`，启动文件内嵌候选位姿 | 地图和赛事 |
| `user_demo/launch/mission.launch` | 将 `user_demo/param/mission.yaml` 加载到 A→B→C→E 任务 | 地图、赛事、标记 ID 及是否在实车运行 |

历史主硬件入口候选为 `abot_bringup/robot.launch`，实际比赛运行命令仍未知。其他历史源码分支包括算术/运算符、VLM 与物体位置分支，导航/识别/发射/语音播报，以及标记对齐后发送附件命令。它们不能确定哪份脚本属于三场赛事中的哪一场。独立 2025 图像集仍在 ROS 2 感知包中，其文件名也不能证明赛事用途。

### ROS 2 能力边界

`abot_mission` 提供显式启动的 Nav2 路线、精确目标结果及可选新鲜目标观测，不复现某场赛事中导航、对齐、发射、播报或 VLM 决策的自动顺序。`abot_accessory` 提供独立显式服务，任务执行不自动调用。用户提供事实前，三条赛事记录继续保持未分配。3×1.5 示意图只作资料，不计划搜索或恢复第三张地图。

<a id="layouts"></a>

## 场地示意图校验

这三张 PNG 是用户在 2026-09-26 提供示意图的原样副本，仅为参考图，**不是**占据栅格或 Nav2 地图服务器输入。`3x3`、`3x1.5` 是用户标注的名义场地尺寸，尚未提供实测比例尺、原点或坐标变换。

| 示意图 | SHA-256 | 关联 |
| --- | --- | --- |
| `field_3x3_targets.png` | `704719d525c5bc640ca4267a70f5d2bb94c6f2c668b1e8d38dd7bbadbea4ec94` | 用户确认对应 `robot_slam/shoot.pgm` |
| `field_3x3_walls_start.png` | `31f40b4f4fb3e9d197c6484474c2eaa65595224aaec224ef1e1ca775c4d1a1bd` | 根据两组 3×3 地图和示意图排除后，暂配 `robot_slam/1.pgm` |
| `field_3x1_5_targets.png` | `85d3671f5c9faec41f2a34f2cecef5baa237b2ca7a1e8c669ded7387dc7cd005` | 此 ABOT 快照中未找到已确认的 PGM/YAML 地图 |

三张示意图都没有与具体赛事名称或任务脚本建立已确认的配对。3×1.5 地图可能已丢失；单凭示意图不能替代实测占据地图。

<a id="mission"></a>

## Nav2 路线任务

本包执行显式配置、顺序进行的 Nav2 `NavigateToPose` 路线。它不启动机器人、不选择地图、不设置初始位姿、不在启动时自动执行路线、不发布 `/cmd_vel`、不执行旧比赛算术逻辑，也不调用外部设备。成功到点后，可选择等待 `Detection2DArray` 中的一类目标。本包不启动 `vision_follower`，也不授予其控制权。

安装的 `config/competition_registry.yaml` 按用户确认保存三条独立的 2025 年赛事记录。其地图、启动文件和任务脚本分配仍未知。该登记表用于盘点，不是本节点接受的路线；旧源码实际确认的文件级关联见 [地图与赛事](TASKS.md#maps)。

### 准备路线

将已安装的 `config/mission.example.yaml` 复制为操作员管理的文件。填写经验证的 `map_id`、`frame_id: map`、显式 `angle_unit`，以及至少一个具名 `navigate` 步骤，坐标须来自地图实测。每步都要设置正数 `timeout_sec`。可选 `observe` 块指定一个绝对 `/perception/...` 话题、精确 `class_id`、正数超时与采集年龄限制，以及最低分数。例如模板检测器发布 `template:10` 这类类别 ID，不会转换为旧数字分组。空示例路线有意设为无效。启动参数 `map_id` 必须与路线 ID 完全一致；这只核对操作员配置，不验证 Nav2 实际加载地图的身份或几何。

```bash
ros2 launch abot_mission mission.launch.py \
  route_file:=/absolute/path/to/validated_mission.yaml \
  map_id:=validated_map_A
# 只有该服务请求会触发路线：
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/status
```

未提供 `route_file` 时，节点保持运行但拒绝 `/mission/start`。已配置节点等待显式启动服务调用。每个目标开始前，要求 Nav2 `bt_navigator` 生命周期状态为 ACTIVE、`/navigate_to_pose` Action 服务端可访问、`map -> base_footprint` TF 足够新。节点异步调用 `/control/acquire_nav`；只有速度控制权管理器能在当前模式为 `disabled` 时原子地授予导航控制权。任务等待成功的租约响应、一个**新的**可靠/瞬态本地 `/control/mode_state=nav` 样本，以及一个**新的**、与返回租约代次匹配的 `/control/mode_epoch` 样本，发送目标前再次检查 Nav2 和定位。操作员的模式命令（包括重复 `nav`）会撤销租约并改变代次；模式或代次变化时，任务取消当前目标。任务自身不发布 `/control/mode`，因此延迟到达的任务停止操作不能覆盖新接受的遥控或跟随模式。`abot_navigation/velocity.launch.py` 中的控制权管理器必须提供这些服务和话题。代次标识该租约所属的控制权世代，单独的 String 状态无法标识一次请求。每步准备受有限的 `preparing_timeout_sec` 限制（默认 10 秒）；缺失 Nav2 生命周期状态、TF 或 Acquire 服务会使该步失败，不会无限等待。

超时或操作员取消时，任务对保存的目标句柄请求取消，不将取消服务响应当作最终目标结果。目标被拒绝或结果不为 SUCCEEDED 时，任务失败或取消；只有精确匹配当前目标的 SUCCEEDED 结果才能推进步骤。目标结束或开始取消后，任务携租约 ID 请求 `/control/release_nav`。释放仍在等待或未确认时，不启动新目标或新任务。释放被拒绝则任务失败，外部控制模式保持不变。即使服务响应先到，下一步仍等待 `disabled` 状态和匹配的释放代次；状态回显缺失时，在有界释放超时后失败，不会继续启动目标。如果 Acquire 超时后才返回租约，任务会释放租约，不发送目标。如果 Acquire 可能已执行但服务 future 失败，租约结果不确定，新工作将被阻止，直到操作员重置控制权管理器和任务节点。无法确认目标结果时，句柄继续保留并拒绝新的启动请求。`/mission/status` 是易失性 JSON String，包含状态、步骤、目标令牌、详情、地图 ID 和租约诊断，用于检查，不是持久化任务日志。

观测是被动的，只有成功释放租约后才开始，仅接受到达时刻或之后采集、且原始图像时间戳足够新的匹配类别。空数组、过期或未来时间戳检测不能推进步骤。启用运动前，必须在实车上验证检测器对应的物理目标身份、相机时钟源、Nav2 地图坐标和定位。

### 离线检查

在 `abot_ros2_ws` 中加载 Humble 环境，然后运行：

```bash
colcon build --packages-up-to abot_mission --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select abot_mission --event-handlers console_direct+
colcon test-result --verbose
```

状态机测试覆盖目标令牌绑定、拒绝、失败、超时、取消和检测新鲜度。节点回调测试使用模拟 Action future 和目标句柄，验证接受、拒绝、取消、结果、延迟服务响应、租约拒绝及传输错误。本地 ROS 图测试使用真实速度控制权服务和话题端点，以及模拟 Nav2 Action 客户端。这些检查不能证明定位精度或实车运动安全。

<a id="accessory"></a>

## 独立附件

本包为旧 `shoot_cmd` 附件提供两个显式 ROS 2 服务入口。一个节点独占附件串口并发送旧协议的 8 字节帧。节点不连接感知话题，任务或视觉检测不会自动调用这些服务。

旧源码选择 `/dev/shoot`、9600 波特率、8 数据位、无校验、1 停止位。协议帧如下：

| 服务 | 帧内容（十六进制） |
| --- | --- |
| `/accessory/shoot` | `55 01 12 00 00 00 01 69` |
| `/accessory/stop` | `55 01 11 00 00 00 01 68` |

协议和实际附件硬件尚未在机器人上测试。这些帧字节直接来自 ROS 1 `shoot_cmd` 源码，离线验证使用伪终端（PTY）。`/dev/shoot` 是附件适配器，与底盘控制器的 `/dev/abot` USB-TTL 端口不同；两个驱动不得配置为占用同一物理适配器。在附件端口启用本节点前，应停止 ROS 1 `shoot_cmd` 进程。

### 默认保护行为与显式调用

默认 `enabled=false`，`port` 为空。此时节点不打开设备、不写入帧，服务被调用时报告关闭状态。启用节点会打开并独占锁定指定端口，但不会发送命令。参数在进程存活期间只读，修改参数需重启节点。

确认适配器路径和硬件测试条件后，显式启用附件：

```bash
ros2 run abot_accessory accessory_driver --ros-args \
  -p enabled:=true -p port:=/dev/shoot -p baud_rate:=9600
```

只有以下显式服务调用会写入命令帧：

```bash
ros2 service call /accessory/shoot std_srvs/srv/Trigger "{}"
ros2 service call /accessory/stop std_srvs/srv/Trigger "{}"
```

每个 `shoot` 服务请求发送一帧发射命令。只要端口已打开，`stop` 就发送停止帧。正常关闭时，仅当节点此前成功发送过发射帧且 `stop_on_shutdown=true`（默认值）时，才尽力发送停止帧，随后关闭端口并释放独占锁。写入错误和断开会返回 `success=false`，响应信息中包含已发送字节数，并输出明确日志。帧只发送一部分便中断时，设备状态不确定；恢复操作前应检查硬件。

这些服务没有请求 ID 或设备确认。再次调用 `shoot` 会再写一帧，因此客户端不能对结果不确定的响应自动重试。`write_timeout_ms` 限制非阻塞写入/轮询循环的时间；后续 `tcdrain` 在异常 USB 串口设备上可能耗时更长。使用前需在静止台架上检查这段时序和机构对重复帧的响应。

### 离线检查

包测试覆盖精确帧字节、9600 8N1 配置、端口独占、伪终端上的服务行为、默认关闭、关闭时发送停止帧以及设备断开。不验证真实附件电路，也不确认设备如何解释旧协议帧。

<a id="voice"></a>

## 语音输入与播报

本包提供 ROS 2 离线语音识别与可选本地语音输出适配器。`voice_transcriber` 接受已写完的 WAV 文件请求，在一个后台工作线程识别，并发布文本和请求状态。它不打开麦克风、不播放音频、不调用网络服务、不解析命令，也不发布运动消息。`voice_announcer` 是独立节点，默认关闭，仅通过本地合成器处理显式文本播报请求。

### 请求约定

将 `audio_root` 配置为音频采集进程可写的收件目录。生产者先写完文件，再原子重命名为最终 `.wav` 文件名。向 `/voice/transcribe/request` 发送 `std_msgs/msg/String` JSON 消息：

```json
{"id":"sample-001","file":"sample.wav"}
```

请求必须恰有两个字段。`id` 为 1–64 个 ASCII 字母、数字、点、下划线或连字符。`file` 必须是以 `.wav` 结尾的基本文件名；拒绝绝对路径、路径分隔符、目录穿越和符号链接。文件必须是普通、非空、未压缩的有符号 16 位 PCM WAV，采样率 16 kHz，单声道或双声道，默认不超过 5 MB 和 60 秒。限制可通过节点参数修改。

工作线程在 `/voice/transcribe/status` 上以 JSON 发布最新状态，例如：

```json
{"id":"sample-001","state":"completed","detail":""}
```

可能状态为 `queued`、`running`、`completed`、`no_speech`、`failed`、`timed_out`、`cancelled`、`rejected`。非空转写通过稳定的 `/voice/text` 话题（`std_msgs/msg/String`）发布为纯文本。状态 ID 用于关联请求；`/voice/text` 有意保持简单文本流，供未来任务层使用。

`queue_size` 限制待处理工作数，活动工作线程只有一个。队列满时请求收到 `rejected`。`request_timeout_sec` 从接受时刻开始计算端到端截止时间，包含队列等待。Python 线程内无法安全中断慢后端调用：工作线程报告 `timed_out`、丢弃迟到文本，直到调用返回才开始下一次识别。单工作线程与有界队列避免慢识别阻塞 ROS 回调或无限累积工作。

### 本地 faster-whisper 后端

首次接受文件时才延迟加载后端。将 `model_path` 配置为已预置且存在的本地模型目录，不接受模型名称或自动下载。可选 Python 包 `faster-whisper` 必须安装在与 ROS 2 相同的 Python 环境。包或模型任一缺失时，请求明确报告 `failed`。默认参数为 `device:=cpu`、`compute_type:=int8`、`language:=zh`、`beam_size:=5`。

示例：

```bash
mkdir -p "$HOME/.ros/abot_voice/inbox"
ros2 run abot_voice voice_transcriber --ros-args \
  -p audio_root:="$HOME/.ros/abot_voice/inbox" \
  -p model_path:=/opt/abot/models/whisper-tiny-zh-ct2-int8
```

本包不附带模型、厂商共享库、凭证或音频资源。安装模型前须检查来源与许可证。`faster-whisper` 是可选项，仅在请求识别时导入。

### 可选本地语音输出

`voice_announcer` 在 `/voice/speak/request` 接受显式请求，在 `/voice/speak/status` 报告每个请求的状态。默认 `enabled:=false`，此时不查找合成器程序、不合成文本、不打开音频设备。仅在需要播放时于启动阶段启用：

```bash
ros2 run abot_voice voice_announcer --ros-args -p enabled:=true
```

发送恰含 `id` 和 `text` 两字段的 `std_msgs/msg/String` JSON 消息：

```json
{"id":"announce-001","text":"机器人已经准备好了。"}
```

ID 使用与转写请求相同的 1–64 字符安全 ASCII 格式。文本必须为非空单行，最多 240 字符、1024 UTF-8 字节。控制字符、无效 Unicode、异常 JSON、多余字段、超大请求、重复活动 ID 和队列溢出均被拒绝。节点只有一个工作线程，默认允许 1 个待处理请求，端到端超时 20 秒（最多可配置为 120 秒）。状态为 `queued`、`running`、`completed`、`failed`、`timed_out`、`cancelled`、`rejected`。

可选后端使用本地安装的开源 `espeak-ng` 程序。节点以 `shell=False` 传入固定参数数组，并把有界文本写入标准输入，不拼接 shell 命令、不运行网络服务、不下载合成器。默认声音为 `zh`，可用 `voice` 参数修改。启用节点前，应将 `espeak-ng` 作为经认可的本地系统包安装。若缺少可执行程序，每个接受请求都明确返回 `failed`。本包不复制合成器程序、语音数据或厂商二进制。当前开发主机未安装 eSpeak NG，因此没有验证实际播放和音质。

### 构建与测试

在 ROS 2 工作区执行：

```bash
colcon build --packages-select abot_voice
source install/setup.bash
pytest src/abot_voice/test
```

测试使用生成的 WAV 和注入的模拟后端，覆盖收件路径限制、WAV 格式校验、有界请求接纳、后台处理、超时及迟到结果处理、缺失本地模型配置、播报文本限制、安全 shell 参数传递和合成器超时。无需麦克风、模型下载、合成器安装或网络。

### 仍需完成的设备工作

- 接入并验证设备端采集进程，使其向配置的收件目录写入已关闭的 WAV；检查实际采样率、声道数、PCM 编码和文件交接行为。
- 在目标计算机上用经认可的本地模型测量 CPU、内存和识别延迟，据此调整限制及模型设置。
- 旧 `robot_voice` 识别器等待 ROS `voiceWakeup` 消息后，才打开麦克风录制固定时长。本包不提供该触发源或麦克风采集，二者的 ROS 2 接入和设备行为仍未验证。
- 在机器人实际声学环境中验证中文识别及无语音行为。未来任务层必须单独定义并验证意图解析和动作策略；`/voice/text` 本身只是文本。
- 旧厂商识别器与播放依赖讯飞库及凭证，其分发权限尚未确认，因此没有复制或启用。

<a id="vlm"></a>

## 按需图像分析

`ros2 launch abot_vlm vlm.launch.py` 启动按需 `/vlm/analyze` 服务端，类型为 `abot_vlm_interfaces/action/AnalyzeImage`，订阅最新 `/camera/image_raw` 帧。启动时不发起推理请求。相机回调只缓存图像；JPEG 转换与 HTTPS 在一个后台工作线程中运行，不写入图像文件，也不发送运动命令。

在节点环境中设置 `ABOT_ARK_API_KEY` 和 `ABOT_ARK_MODEL` 即可启用可选 Ark HTTPS 后端。两者都不记录日志或保存；旧模型 ID 与凭证文件未复制。API 端点为 `https://ark.cn-beijing.volces.com/api/v3/chat/completions`。

目标字段：`prompt` 必填，长度 1–4096 字符；`max_frame_age_sec` 或 `timeout_sec` 设为零时采用节点默认值（分别为 1 秒、15 秒）。请求的帧年龄超过 `maximum_frame_age_sec`（默认 5 秒）会被拒绝，超时超过 `max_timeout_sec`（默认 60 秒）会被截断到上限。节点构建时四个限制参数都必须是有限正数；最大年龄上限 60 秒，最大超时上限 120 秒。帧年龄按本地单调接收时间计算，因此相机时钟不一致不会让旧帧看似新鲜。同一时间只接受一个请求。取消或超时会终止对应 Ark HTTPS 子进程，立即返回 Action 结果；工作线程清理完成后才接受新目标，迟到答案丢弃。注入的自定义后端可能无法立即停止，其工作线程租约会一直保持到退出。

Action 结果使用 `code` 和 `text`，仅 `OK` 时 `text` 含答案。其他代码为 `NO_FRAME`、`STALE_FRAME`、`UNAVAILABLE`、`TIMEOUT`、`BACKEND_ERROR`、`CANCELED` 和 `INVALID_IMAGE`。ROS Action 拒绝目标表示请求字段无效或工作线程忙。目标 UUID 标识请求，每个被接受目标恰好有一个最终 Action 结果。

只接受 `mono8`、`rgb8`、`bgr8` 相机编码，图像限制为 400 万像素。实现已用模拟后端与本地 ROS Action 客户端验证。硬件相机模式、远程 Ark 服务、凭证及真实回答质量仍需台架验证。
