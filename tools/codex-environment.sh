#!/usr/bin/env bash
# Codex 本地环境操作；所有入口均为无设备开发/验证。
set -eo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
abot_action="${1:-check}"
case "$abot_action" in
  check|build|verify|maps) ;;
  *) echo '用法: bash tools/codex-environment.sh {check|build|verify|maps}' >&2; exit 2 ;;
esac

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '需要 Linux/WSL Ubuntu 22.04 与 ROS 2 Humble；请按 docs/DEPLOYMENT.md 安装。' >&2
  exit 1
fi
if [[ -n "${CONDA_PREFIX:-}" || "${ROS_VERSION:-2}" != 2 ||
      ( -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ) ]]; then
  echo '请从未加载 Conda、ROS1 或其他 ROS 发行版的 Bash 运行。' >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
if [[ -f .validation/deps/install/setup.bash ]]; then
  source deployment/use-validation-deps.bash
fi
if [[ -d .vendor/ydlidar-sdk-install ]]; then
  export CMAKE_PREFIX_PATH="$PWD/.vendor/ydlidar-sdk-install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LIBRARY_PATH="$PWD/.vendor/ydlidar-sdk-install/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
fi

export PYTHONUNBUFFERED=1
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
# 本入口用于本机离线验证；生产多机网络按部署文档另行设置。
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-166}"
if [[ -f .validation/cyclone-deps/install-rmw/local_setup.bash ]]; then
  source .validation/cyclone-deps/install-rmw/local_setup.bash
  export CMAKE_PREFIX_PATH="$PWD/.validation/cyclone-deps/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$PWD/.validation/cyclone-deps/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://$PWD/deployment/cyclonedds-local-test.xml"
  unset FASTRTPS_DEFAULT_PROFILES_FILE
fi

case "$abot_action" in
  check)
    abot_missing=0
    for abot_command in colcon cmake python3 ros2; do
      if ! command -v "$abot_command" >/dev/null; then
        echo "缺少命令：$abot_command" >&2
        abot_missing=1
      fi
    done
    for abot_package in ament_cmake robot_localization imu_filter_madgwick twist_mux nav2_bringup slam_toolbox usb_cam laser_filters vision_msgs cv_bridge; do
      if ! ros2 pkg prefix "$abot_package" >/dev/null 2>&1; then
        echo "缺少运行依赖：$abot_package" >&2
        abot_missing=1
      fi
    done
    if ! ros2 pkg prefix ydlidar_ros2_driver >/dev/null 2>&1 &&
       [[ ! -f src/ydlidar_ros2_driver/package.xml ]]; then
      echo '缺少固定版本 YDLIDAR 驱动（依赖叠加层或 src/ydlidar_ros2_driver）。' >&2
      abot_missing=1
    fi
    if (( abot_missing )); then
      echo '按 docs/DEPLOYMENT.md 准备依赖后重试；此检查不自动安装系统包。' >&2
      exit 1
    fi
    printf 'ROS2 环境检查通过：%s\nROS_DOMAIN_ID=%s，ROS_LOCALHOST_ONLY=%s\n' "$ROS_DISTRO" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY"
    echo '这是主要依赖检查；完整构建与测试请运行 verify。'
    ;;
  build)
    colcon build --base-paths src --parallel-workers 1 --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    ;;
  verify)
    bash tools/verify.sh
    ;;
  maps)
    python3 src/abot_maps/test/test_installed_maps.py
    colcon build --base-paths src --packages-select abot_maps --parallel-workers 1
    colcon test --packages-select abot_maps --event-handlers console_direct+ --return-code-on-test-failure
    colcon test-result --test-result-base build/abot_maps --verbose
    ;;
esac
