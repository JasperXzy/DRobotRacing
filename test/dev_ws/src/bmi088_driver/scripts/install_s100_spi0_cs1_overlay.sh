#!/usr/bin/env bash

set -euo pipefail

readonly overlay_name="bmi088_spi0_cs1.dtbo"
readonly overlay_boot_path="/boot/overlays/${overlay_name}"
readonly config_path="/boot/config.txt"
readonly config_line="dtbo_file_path=/overlays/${overlay_name}"
temporary_overlay=""
overlay_test_result=""

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

require_root() {
  [[ "$(id -u)" -eq 0 ]] || fail "run this command with sudo"
}

check_board() {
  [[ -r /sys/firmware/devicetree/base/compatible ]] ||
    fail "cannot read the running device tree"
  if ! tr '\0' '\n' </sys/firmware/devicetree/base/compatible | grep -qx 'drobot,s100-rdk'; then
    fail "this overlay is only for an RDK S100"
  fi
  [[ -e /sys/firmware/devicetree/base/__symbols__/spi0 ]] ||
    fail "the running device tree has no spi0 symbol"
}

cleanup() {
  if [[ -n "$temporary_overlay" ]]; then
    rm -f -- "$temporary_overlay"
  fi
  if [[ -n "$overlay_test_result" ]]; then
    rm -f -- "$overlay_test_result"
  fi
}

remove_overlay() {
  require_root
  [[ -f "$config_path" ]] || fail "$config_path does not exist"
  sed -i "\|^${config_line}$|d" "$config_path"
  if [[ -e "$overlay_boot_path" ]]; then
    rm -- "$overlay_boot_path"
  fi
  sync
  echo "BMI088 SPI0 CS1 overlay removed. Reboot to return to the previous device tree."
}

install_overlay() {
  require_root
  check_board
  command -v dtc >/dev/null || fail "install device-tree-compiler first: sudo apt install device-tree-compiler"
  command -v fdtoverlay >/dev/null || fail "fdtoverlay is required (provided by device-tree-compiler)"

  local script_directory
  script_directory="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
  local overlay_source="${script_directory}/../dts/s100_spi0_cs1_overlay.dts"
  if [[ ! -r "$overlay_source" ]]; then
    overlay_source="${script_directory}/../../share/bmi088_driver/dts/s100_spi0_cs1_overlay.dts"
  fi
  [[ -r "$overlay_source" ]] || fail "overlay source is missing: $overlay_source"
  [[ -f "$config_path" ]] || fail "$config_path does not exist"

  if grep -q '^dtbo_file_path=' "$config_path" && ! grep -qxF "$config_line" "$config_path"; then
    fail "$config_path already loads another DTBO; merge the overlays instead of replacing it"
  fi

  temporary_overlay="$(mktemp /tmp/bmi088-spi0-cs1-XXXXXX.dtbo)"
  overlay_test_result="$(mktemp /tmp/bmi088-spi0-cs1-test-XXXXXX.dtb)"
  trap cleanup EXIT

  dtc -@ -I dts -O dtb -o "$temporary_overlay" "$overlay_source"
  fdtoverlay -i /sys/firmware/fdt -o "$overlay_test_result" "$temporary_overlay"

  local backup_path="${config_path}.bmi088-backup-$(date +%Y%m%d%H%M%S)"
  cp --preserve=mode,ownership,timestamps "$config_path" "$backup_path"
  install -d -m 0755 /boot/overlays
  install -m 0644 "$temporary_overlay" "$overlay_boot_path"
  if ! grep -qxF "$config_line" "$config_path"; then
    printf '\n%s\n' "$config_line" >>"$config_path"
  fi
  sync
  cleanup
  trap - EXIT

  echo "Installed ${overlay_boot_path}"
  echo "Backed up ${config_path} to ${backup_path}"
  echo "Reboot, then check: ls -l /dev/spidev0.0 /dev/spidev0.1"
}

case "${1:-install}" in
  install)
    install_overlay
    ;;
  remove|--remove)
    remove_overlay
    ;;
  *)
    fail "usage: $0 [install|--remove]"
    ;;
esac
