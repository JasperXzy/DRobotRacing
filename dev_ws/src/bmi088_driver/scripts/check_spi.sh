#!/usr/bin/env bash

set -u

accel_device="${1:-/dev/spidev0.0}"
gyro_device="${2:-/dev/spidev0.1}"
failed=0

check_device() {
  local label="$1"
  local device="$2"
  if [[ ! -e "$device" ]]; then
    echo "ERROR: $label device is missing: $device"
    failed=1
    return
  fi
  if [[ ! -r "$device" || ! -w "$device" ]]; then
    echo "ERROR: $label device is not readable and writable by $(id -un): $device"
    failed=1
    return
  fi
  echo "OK: $label device is accessible: $device"
}

check_device "accelerometer" "$accel_device"
check_device "gyroscope" "$gyro_device"

echo "Kernel spidev devices:"
if [[ -d /sys/class/spidev ]]; then
  find /sys/class/spidev -maxdepth 1 -mindepth 1 -printf '  %f\n' | sort
else
  echo "  none"
fi

if [[ "$gyro_device" == "/dev/spidev0.1" && ! -e "$gyro_device" && -e /sys/class/spidev/spidev1.0 ]]; then
  echo "NOTE: spidev1.0 is SPI1 chip-select 0, not SPI0 chip-select 1."
  echo "      Do not substitute it unless the BMI088 is physically wired to the SPI1 pins."
fi

if [[ "$failed" -ne 0 ]]; then
  echo "SPI preflight failed. Enable/map both SPI0 chip selects before starting bmi088_driver."
  exit 1
fi

echo "SPI preflight passed. The ROS driver will perform chip-ID and configuration checks."
