# ABOT ROS 2 项目开发约定

## 范围与沟通

- 默认使用简体中文回复与维护文档；保留代码标识符、命令、话题和配置键的原名。
- 从 Git 根目录执行命令，使用当前检出的路径，不写死开发者主目录。此仓库是合并后的唯一 ROS2 工作区；不要恢复旧 `abot_ws`、`catkin_ws` 或 ROS1 源码依赖。
- 先读 `README.md` 和与任务相关的主题文档。当前状态以 `docs/MIGRATION_STATUS.md` 为入口，接口以 `docs/INTERFACES.md` 为准；实现与文档冲突时核对源码和测试。
- 保护用户已有改动。正常的源码、配置、文档修改及离线验证在任务授权范围内直接完成；有缺失信息时先推进不依赖该信息的工作。

## 目录与环境

- `src/`：15 个项目包；`deployment/`：部署配置与固定依赖；`tools/`：验证工具。
- `docs/` 只维护现有 8 份主题文档。优先修改对应章节，不为每次任务增加计划、交接、总结或逐包 README。
- 使用 Ubuntu 22.04、ROS 2 Humble、系统 Python 3.10、C++17、ament/colcon。不要混入 catkin、Conda 或其他 ROS 发行版环境。
- 普通环境先 `source /opt/ros/humble/setup.bash`；本机已有 WSL 验证依赖时使用 `source deployment/use-validation-deps.bash`。缺依赖时按 `docs/DEPLOYMENT.md` 处理，不猜固定提交、不静默改为最新版。
- `build/`、`install/`、`log/`、`.validation/`、`.vendor/` 为本地生成内容，不提交。不要将仓库 `.codex/` 设置成 `CODEX_HOME`，也不要在其中保存登录凭据、会话或个人配置。

## 实现与硬件边界

- 保留麦克纳姆底盘的 `vx/vy/wz`。最终 `/cmd_vel` 只能由 `velocity_gate` 发布；任务、跟随和遥控通过已有命令源、控制权及门控链进入底盘。
- 保持 TF 单一发布职责：EKF 发布 `odom → base_footprint`，SLAM 或 AMCL 发布 `map → odom`。不要用静态变换或虚构数据绕过缺失的状态估计。
- 默认启动保持设备关闭。普通开发/测试使用合成输入、伪终端和无设备烟测；实际打开串口、激活底盘、控制舵机或附件，应属于用户明确要求的实物调试范围。
- ABOT 与机械臂组合入口管理同一套底盘节点，不能同时启动完整入口。
- 机械臂为五个运动关节加夹爪；串口写入不等于实测到位。不要把指令角度当 `/joint_states`，不要用旧固定相机 TF 代替腕部标定。设备参数和已验证边界见 `docs/MOBILE_MANIPULATOR.md`。
- 不通过放宽故障停车、新鲜度、取消结果、进程清理或标定门控来使测试通过。

## 验证与资源校验

- 先运行受影响包的现有测试；接口或跨包行为变更再运行相关集成检查。文档或 Codex 配置变更只做对应的语法、链接和差异检查，无需重跑全部 ROS 测试。
- 完整构建与离线检查入口为 `bash tools/verify.sh`。WSL 的 Cyclone 环境、严格 P0 双次运行及实车验收规则见 `docs/VALIDATION.md`；不要把仅测试用的 DDS/bond 参数改成生产默认值。
- 针对单包，在已加载依赖的 Bash 中执行（将 `包名` 替换为实际包名）：

  ```bash
  colcon build --packages-up-to 包名 --parallel-workers 1
  source install/setup.bash
  colcon test --packages-select 包名 --event-handlers console_direct+ --return-code-on-test-failure
  colcon test-result --test-result-base build/包名 --verbose
  ```

- 地图或模板文件的任何字节修改（包括尾部空行和换行符）都会影响 SHA-256。修改前确认清单约定，同步更新对应安装文件哈希；保留历史源文件哈希，不能用当前文件哈希覆盖来源证据。不要为消除格式警告而批量重写资源文件。
- 地图源码快速检查：`python3 src/abot_maps/test/test_installed_maps.py`。修改地图/YAML/清单/安装规则后，还必须构建并测试 `abot_maps`，确认安装态 `installed_map_pairs` 通过。
- 修复回归时先复现，说明根因，执行能覆盖故障路径的检查。报告实际运行的命令、结果及未验证范围，不把文档中的历史测试数当成本次结果。
- `.validation/` 是本地证据，可能不随仓库分发；不存在时不能声称已查验。离线启动、速度传播或目标取消通过，不等于实车导航到点或自动抓取验收。

## 提交与审查

- 提交前检查实际暂存差异和 `git diff --cached --check`，排除凭据、模型权重和构建产物。按照用户授权进行提交和推送，不因本文件自动发布改动。
- 行为、接口或部署流程变化时更新对应主题文档。报告修改内容、验证结果和仍待实测的事项。

## Code Review Rules

- 检查速度发布者、导航租约、目标取消、超时停车和生命周期关闭是否保持现有约定。
- 检查资源文件与清单哈希是否一致，新增资源能否正确安装；地图校验必须覆盖安装目录。
- 检查设备默认关闭和标定拒绝路径，避免新增隐式硬件动作、虚构状态反馈或未经标定的变换。
