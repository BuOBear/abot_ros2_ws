# 部署前置条件

## Codex 项目环境

项目环境文件为 `.codex/environments/environment.toml`，操作入口为 `tools/codex-environment.sh`。在 Codex 中打开此 Linux/WSL 项目后，可使用“检查 ROS2 环境”“构建工作区”“完整离线验证”和“地图校验”操作。新工作树的 setup 执行依赖检查，缺依赖时给出错误；按本文安装依赖后重试。它不自动安装系统包或开启设备。

每个操作独立加载 Humble，存在时复用本工作区的 `.validation/deps` 和 Cyclone 叠加层，并识别 `.vendor/ydlidar-sdk-install`。新工作树不会自动复制原工作区中被 Git 忽略的依赖，需按本文恢复或在目标主机安装。初始化过程中的 `source` 不会持久改变后续终端，因此操作通过同一脚本重新加载环境。

```bash
bash tools/codex-environment.sh check
bash tools/codex-environment.sh build
bash tools/codex-environment.sh verify
bash tools/codex-environment.sh maps
```

这些入口仅用于本机离线开发，设置 `ROS_LOCALHOST_ONLY=1`，默认 ROS 域为 166。并发验证须使用不同且空闲的域，例如 `ROS_DOMAIN_ID=167 bash tools/codex-environment.sh verify`。`verify` 调用现有 `tools/verify.sh`，不包含完整 P0 双次严格集成；该验收另见 [验证说明](VALIDATION.md)。真机和多机通信按后续部署步骤操作。

## 目标主机依赖

目标环境为 Ubuntu 22.04 / ROS 2 Humble、系统 Python 3.10。原 ABOT ROS1 源码树已按用户要求删除，catkin ROS1 源码也已移除。不要在此工作区环境中同时加载 catkin/Conda，只构建本工作区的 `src`。

在已启用 ROS apt 软件源的目标主机上执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep v4l-utils
# sudo rosdep init  # 仅在主机从未初始化过 rosdep 时执行
rosdep update --rosdistro humble
```

已报告目标为 Ubuntu 22.04 x86 小主机。验证工作区前，先按固定提交构建官方 YDLidar-SDK 和驱动。在 `abot_ros2_ws` 中运行以下命令；SDK 安装在当前工作区本地，驱动导入 `src`：

```bash
mkdir -p .vendor
git clone https://github.com/YDLIDAR/YDLidar-SDK.git .vendor/YDLidar-SDK
git -C .vendor/YDLidar-SDK checkout 42a82ed10d2304094c111fc63dee8e4a229b79b7
cmake -S .vendor/YDLidar-SDK -B .vendor/YDLidar-SDK/build \
  -DCMAKE_INSTALL_PREFIX="$PWD/.vendor/ydlidar-sdk-install" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF -DBUILD_SHARED_LIBS=OFF
cmake --build .vendor/YDLidar-SDK/build --parallel 2
cmake --install .vendor/YDLidar-SDK/build
git clone --branch humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git src/ydlidar_ros2_driver
git -C src/ydlidar_ros2_driver checkout 4ef70d3f32a85704ade0be54b214f3763b1ab3e8
export CMAKE_PREFIX_PATH="$PWD/.vendor/ydlidar-sdk-install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LIBRARY_PATH="$PWD/.vendor/ydlidar-sdk-install/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
bash tools/verify.sh
```

若源码已存在，应检查提交版本，不要重复克隆。相同固定版本驱动也记录在 `deps.repos`。开发用 WSL 主机缺少 `robot_localization`、`imu_filter_madgwick`、`twist_mux`，且没有免密 sudo，其本地验证叠加层见本文后半部分。可选本地播报需在启用 `voice_announcer` 前于目标机单独安装 `espeak-ng`；播报器默认关闭，本开发主机尚未验证音频播放。

验收前记录目标环境版本：

```bash
dpkg-query -W 'ros-humble-*' > deployment/target-ros-packages.txt
uname -a > deployment/target-kernel.txt
```

## 需在实车完成的设备与网络盘点

已报告底盘 MCU 为通过 USB 转 TTL 连接的 STM32F103VET6。雷达为 YDLIDAR X3 Pro；官方[型号表](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/Dataset.md)列出 115200 波特率、3 kHz 采样、0.10–8 m 量程和 4–8 Hz PWM 扫描。[官方 X3 ROS2 配置](https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/humble/params/X3.yaml)提供了本项目起始驱动参数。已报告相机为 USB OV5640，200 万像素，对角视场 148°，标称最高 120 fps。按用户要求，旧 640×480 `track_tag/camera_calibration.yaml` 已安装并通过 `camera_info_url` 默认选择；可用该启动参数指定其他文件。实际采集模式和 Image/CameraInfo 对齐仍需设备端检查。机器人静止时记录：

```bash
bash tools/inventory_target.sh > target-inventory.txt
# 也可手动检查各项信息：
uname -m
lsb_release -ds
ls -l /dev/serial/by-id/ /dev/serial/by-path/ 2>/dev/null
lsusb
udevadm info --query=property --name=/dev/ttyUSB0  # 对每个候选端口重复执行
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0  # 使用上面找到的设备
```

识别底盘和雷达各自的串口适配器，再分别配置稳定链接 `/dev/abot` 与 `/dev/ydlidar`。这些是预期配置名，并非已确认存在的设备。只有通过 `udevadm info --attribute-walk --name=<device>` 确定唯一属性后才创建 udev 规则；单靠厂商/产品 ID 可能冲突。赋予部署用户串口/视频组权限，避免端口对所有用户可写。旧底盘 YAML 建议 921600，但须由适配器及 STM32 固件确认。旧 `rplidar.launch` 与已确认的 X3 Pro 不兼容，不得用于此机器人。启用传感器前，在 `abot_bringup/config/lidar.yaml` 和 `camera.yaml` 填写实测端口、雷达电机控制和相机模式，再用实际设备验证扫描时间戳、角覆盖和 CameraInfo。

记录轮序、滚子朝向及尺寸、已烧录固件版本、MCU 断线/断电停车时间、雷达/相机外参和历史 ROS1 启动命令。历史主硬件入口候选为 `abot_bringup/robot.launch`，导航入口为 `robot_slam/navigation.launch`；实际运行命令仍待核实。

选择并记录 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION` 和多机时钟同步方式。集成测试使用本机回环与隔离域。

## 台架激活与回退

底盘关闭、传感器端口及模式选定后，接入导航前分别检查两路传感器流：

```bash
ros2 launch abot_bringup bringup.launch.py enable_lidar:=true
ros2 topic info -v /scan
ros2 topic hz /scan
ros2 topic echo --once /scan --field header --qos-reliability best_effort
ros2 topic info -v /scan_filtered
# 选定实际 V4L2 模式后，另行运行：
ros2 launch abot_bringup bringup.launch.py enable_camera:=true
ros2 topic info -v /camera/image_raw
ros2 topic info -v /camera/camera_info
```

根据 `abot_navigation/config/velocity.yaml` 保护条件检查完整扫描角覆盖和有效射线比例，并确认所选旧 640×480 CameraInfo 与实时图像尺寸及坐标系一致。示例命令只启动描述和选定传感器，不激活底盘。

`$(ros2 pkg prefix abot_maps)/share/abot_maps/maps/robot_slam` 仅安装 `1.pgm`/`1.yaml` 和 `shoot.pgm`/`shoot.yaml`。定位时显式选择 YAML。来源清单记录历史来源，但不依赖已删除的 ROS1 源码树。

只有在选定实物配置且速度链已验证后，才显式启动硬件。底盘初始为未配置；设置验证完成后，`ros2 lifecycle set /abot_hardware configure` 和 `activate` 是需要明确执行的台架步骤。不要同时启动 ROS1 与 ROS2 硬件驱动。用户报告 MCU 会在主机断电或串口丢失时立即停电机；运动验收前，应在车轮卸载条件下实测两种路径。这是 M2 测试门槛。

ROS1 源码和根构建缓存已不再包含。ROS2 不会自动覆盖 MCU 调参。离线测试通过不能解释为完成实测标定或真机导航验收。

## 移动机械臂

`jy_real_interfaces` 与 `jy_real_robot` 已纳入同一源码树和验证命令，详见[模块部署](MOBILE_MANIPULATOR.md)。可选 YOLO11 运行环境和本地权重需单独准备；普通构建/测试不安装 Torch、不下载权重、不启用设备。使用机械臂组合启动入口，不要同时启动完整 ABOT 入口。

## WSL 本地验证依赖

此临时叠加层提供当前 Ubuntu 22.04 环境缺失的 ROS 包。YDLIDAR ROS2 驱动从官方 Humble 分支构建，链接另行固定版本的 YDLidar-SDK。源码版本固定在 [validation-deps.repos](../deployment/validation-deps.repos)；配置完善的目标主机可使用其他依赖的 Humble 二进制包。

在 `abot_ros2_ws` 中按以下命令恢复：

```bash
mkdir -p .validation/deps/src .validation/deps/system
cd .validation/deps/src
git clone https://github.com/ros-geographic-info/geographic_info.git
git -C geographic_info checkout f70b81a438172cd7a066dc1b18314d70e0eb6389
git clone https://github.com/CCNYRoboticsLab/imu_tools.git
git -C imu_tools checkout 1711f25e8af75231174a725f1023f915bd5c64a3
git clone https://github.com/cra-ros-pkg/robot_localization.git
git -C robot_localization checkout 8696ee5a9e4f959fcaae37835dcf2ed12ead581b
git clone --branch humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git
git -C ydlidar_ros2_driver checkout 4ef70d3f32a85704ade0be54b214f3763b1ab3e8
git clone https://github.com/ros-teleop/twist_mux.git
git -C twist_mux checkout d929f1b19a437abcf131287b9e3af27fb40ed34e
cd ../system
apt-get download libgeographic-dev libgeographic19
for deb in *.deb; do dpkg-deb -x "$deb" .; done
cd ..
git clone https://github.com/YDLIDAR/YDLidar-SDK.git YDLidar-SDK
git -C YDLidar-SDK checkout 42a82ed10d2304094c111fc63dee8e4a229b79b7
cmake -S YDLidar-SDK -B YDLidar-SDK/build \
  -DCMAKE_INSTALL_PREFIX="$PWD/sdk-install" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF -DBUILD_SHARED_LIBS=OFF
cmake --build YDLidar-SDK/build --parallel 2
cmake --install YDLidar-SDK/build
source /opt/ros/humble/setup.bash
export CPLUS_INCLUDE_PATH="$PWD/system/usr/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
export CMAKE_PREFIX_PATH="$PWD/sdk-install:$PWD/system/usr${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LIBRARY_PATH="$PWD/sdk-install/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$PWD/system/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --base-paths src \
  --packages-select geographic_msgs robot_localization imu_filter_madgwick twist_mux ydlidar_ros2_driver \
  --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF \
  -DCMAKE_MODULE_PATH="$PWD/system/usr/share/cmake/geographiclib"
cd ../..
```

构建或运行项目工作区前，加载已验证的环境脚本：

```bash
source deployment/use-validation-deps.bash
```

`use-validation-deps.bash` 加载 `/opt/ros/humble` 与已构建的叠加层，并加入解包的 GeographicLib 运行库。固定版本 `robot_localization` 源码未添加 GeographicLib 头文件目录，因此需导出 `CPLUS_INCLUDE_PATH`。YDLIDAR 驱动的 CMake 读取 SDK 头文件路径，但 SDK 包配置未提供库搜索路径，因此本地静态 SDK 链接需要 `LIBRARY_PATH`。叠加层位于 `.validation`，不修改系统包。

验证使用解包的 Ubuntu `libgeographic-dev` 和 `libgeographic19`，均为 amd64 版本 `1.52-1`，SHA-256 为：

```text
33130034ffdf72478b89c996830a7411de7942027efc29e4b8ba76a6e6cedb12  libgeographic-dev_1.52-1_amd64.deb
143c1b80105e2b72ad492de6e91abc5f26dbe0197e5e4f4ecedf4392d201ab83  libgeographic19_1.52-1_amd64.deb
```


## WSL 的 Cyclone DDS 叠加层


使用第二套本地 RMW 叠加层后，此 WSL 主机通过了严格 P0 离线测试。版本为 Cyclone DDS 0.10.5、提交 `5041f3560c088c99e5088b2b8520b69169621196`，以及 Humble `rmw_cyclonedds_cpp` 1.3.5、提交 `e370e09ca76fc811e42ff07bd3a5e3b92f18c51e`。安装普通验证依赖后，在 `abot_ros2_ws` 恢复该叠加层：

```bash
mkdir -p .validation/cyclone-deps/src
git clone --branch releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git \
  .validation/cyclone-deps/src/cyclonedds
git -C .validation/cyclone-deps/src/cyclonedds checkout \
  5041f3560c088c99e5088b2b8520b69169621196
git clone --branch humble https://github.com/ros2/rmw_cyclonedds.git \
  .validation/cyclone-deps/src/rmw_cyclonedds
git -C .validation/cyclone-deps/src/rmw_cyclonedds checkout \
  e370e09ca76fc811e42ff07bd3a5e3b92f18c51e
cmake -S .validation/cyclone-deps/src/cyclonedds \
  -B .validation/cyclone-deps/build-cyclonedds -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/.validation/cyclone-deps/install" \
  -DENABLE_SHM=OFF -DBUILD_DDSPERF=OFF -DBUILD_TESTING=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_IDLC=ON
cmake --build .validation/cyclone-deps/build-cyclonedds --parallel 2
cmake --install .validation/cyclone-deps/build-cyclonedds
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$PWD/.validation/cyclone-deps/install:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$PWD/.validation/cyclone-deps/install/lib:$LD_LIBRARY_PATH"
colcon --log-base .validation/cyclone-deps/log-rmw build \
  --base-paths .validation/cyclone-deps/src/rmw_cyclonedds \
  --packages-select rmw_cyclonedds_cpp \
  --build-base .validation/cyclone-deps/build-rmw \
  --install-base .validation/cyclone-deps/install-rmw \
  --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=Release
```


这些依赖只安装到 `.validation/`。运行环境、传输配置限制和测试命令见 [验证与验收](VALIDATION.md)。
