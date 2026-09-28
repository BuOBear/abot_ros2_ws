#!/usr/bin/env bash
# Read-only inventory for the robot's Ubuntu 22.04 mini PC.
set -uo pipefail

section() { printf '\n[%s]\n' "$1"; }

section 'host'
if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  printf 'OS=%s\n' "${PRETTY_NAME:-unknown}"
fi
printf 'architecture=%s\n' "$(uname -m)"
printf 'kernel=%s\n' "$(uname -r)"
if [[ -r /proc/cpuinfo ]]; then
  awk -F: '/^model name[[:space:]]*:/ {sub(/^[[:space:]]+/, "", $2); print "CPU=" $2; exit}' /proc/cpuinfo
fi
if [[ -r /proc/meminfo ]]; then
  awk '/^MemTotal:/ {print "RAM_kB=" $2}' /proc/meminfo
fi
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-unset}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-unset}"

section 'USB devices'
if command -v lsusb >/dev/null 2>&1; then
  lsusb
  if ! lsusb -t 2>&1; then
    printf 'USB topology unavailable\n'
  fi
else
  printf 'lsusb unavailable\n'
fi

section 'stable serial links'
for directory in /dev/serial/by-id /dev/serial/by-path; do
  if [[ -d "$directory" ]]; then
    ls -l "$directory"
  else
    printf '%s absent\n' "$directory"
  fi
done

section 'USB serial adapters'
serial_count=0
for device in /dev/ttyUSB* /dev/ttyACM*; do
  [[ -e "$device" ]] || continue
  serial_count=$((serial_count + 1))
  printf '\n%s\n' "$device"
  if command -v udevadm >/dev/null 2>&1; then
    udevadm info --query=property --name="$device" 2>&1 | awk -F= '
      $1 ~ /^(DEVNAME|DEVPATH|ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|ID_SERIAL_SHORT|ID_PATH|ID_USB_INTERFACE_NUM)$/ {print}'
  else
    printf 'udevadm unavailable\n'
  fi
done
if (( serial_count == 0 )); then
  printf 'No ttyUSB or ttyACM devices found\n'
fi

section 'video devices and modes'
for directory in /dev/v4l/by-id /dev/v4l/by-path; do
  if [[ -d "$directory" ]]; then
    ls -l "$directory"
  else
    printf '%s absent\n' "$directory"
  fi
done
if command -v v4l2-ctl >/dev/null 2>&1; then
  video_count=0
  for device in /dev/video*; do
    [[ -e "$device" ]] || continue
    video_count=$((video_count + 1))
  done
  if (( video_count == 0 )); then
    printf 'No video devices found\n'
  else
    v4l2-ctl --list-devices 2>&1
  fi
  for device in /dev/video*; do
    [[ -e "$device" ]] || continue
    printf '\n%s\n' "$device"
    v4l2-ctl --list-formats-ext -d "$device" 2>&1
  done
else
  printf 'v4l2-ctl unavailable; install v4l-utils\n'
fi
