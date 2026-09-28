#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
colcon build --base-paths src --parallel-workers 1 --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test --packages-select abot_hardware abot_description abot_navigation abot_bringup abot_maps abot_perception abot_tracking abot_mission abot_accessory abot_voice abot_control_interfaces abot_vlm_interfaces abot_vlm jy_real_interfaces jy_real_robot \
  --parallel-workers 1 --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
ros2 launch abot_bringup bringup.launch.py --show-args
ros2 launch abot_bringup sensors.launch.py --show-args
ros2 launch abot_mission mission.launch.py --show-args
ros2 launch abot_vlm vlm.launch.py --show-args
ros2 launch abot_tracking lidar_follow.launch.py --show-args
ros2 launch abot_tracking vision_follow.launch.py --show-args
ros2 launch abot_tracking roi_track.launch.py --show-args
ros2 launch abot_perception fire_detector.launch.py --show-args
ros2 launch jy_real_robot real.launch.py --show-args
python3 tools/smoke_description.py
python3 tools/smoke_manipulator.py --mode default --output .validation/manipulator-default
