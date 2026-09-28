# Source this file from Bash after restoring the pinned overlay in .validation/deps.
_abot_validation_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/.validation/deps"
if [[ ! -f "$_abot_validation_root/install/setup.bash" ]]; then
  echo 'Validation dependencies are not built; see docs/DEPLOYMENT.md.' >&2
  unset _abot_validation_root
  return 1
fi
if [[ ! -f "$_abot_validation_root/sdk-install/lib/libydlidar_sdk.a" ||
      ! -x "$_abot_validation_root/install/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node" ]]; then
  echo 'Pinned YDLIDAR SDK/driver is missing; see docs/DEPLOYMENT.md.' >&2
  unset _abot_validation_root
  return 1
fi
source /opt/ros/humble/setup.bash
source "$_abot_validation_root/install/setup.bash"
export CMAKE_PREFIX_PATH="$_abot_validation_root/sdk-install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LIBRARY_PATH="$_abot_validation_root/sdk-install/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$_abot_validation_root/system/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
unset _abot_validation_root

