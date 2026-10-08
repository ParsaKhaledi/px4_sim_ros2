#!/bin/bash
# Write the sim-only PX4 parameter profile as PX4_PARAM_* assignments.
#
# PX4 v1.17 rcS applies every PX4_PARAM_<NAME> from the environment
# before the airframe and before ekf2 start:
#   ROMFS/px4fmu_common/init.d-posix/rcS
# Appending "param set" to px4-rc.params does not work. That file is
# not sourced. gz_start_px4_gz_sim.sh sources the file this writes.
#
# NAV_DLL_ACT=0 is sim-only. The x500 airframe default is 2, which
# blocks arming when headless SITL has no QGroundControl.
#
# Usage: install_px4_control_params <env-file> [vision|gps]
#
# Vision-mode environment overrides (see docs/px4_control.md):
#   EKF2_EV_DELAY   milliseconds, 0..300, default 0
#   EKF2_EV_CTRL    integer bitmask 0..15, default 11 (no velocity)

_px4_control_number() {
  local name="$1"
  local raw="$2"
  local min="$3"
  local max="$4"
  local integer="${5:-0}"
  awk -v name="$name" -v raw="$raw" -v min="$min" -v max="$max" -v integer="$integer" 'BEGIN {
    if (raw + 0 != raw) { printf "%s is not a number: %s\n", name, raw > "/dev/stderr"; exit 1 }
    value = raw + 0
    if (integer && value != int(value)) { printf "%s must be an integer, got %s\n", name, raw > "/dev/stderr"; exit 1 }
    if (value < min + 0 || value > max + 0) { printf "%s must be in [%s, %s], got %s\n", name, min, max, raw > "/dev/stderr"; exit 1 }
    print value
  }'
}

install_px4_control_params() {
  local env_file="$1"
  local mode="${2:-${ESTIMATION_MODE:-vision}}"
  mode="$(printf '%s' "$mode" | tr '[:upper:]' '[:lower:]')"
  if [ "$mode" != "vision" ] && [ "$mode" != "gps" ]; then
    echo "install_px4_control_params: ESTIMATION_MODE must be vision or gps, got '$mode'" >&2
    return 1
  fi
  local dir
  dir="$(dirname "$env_file")"
  if [ ! -d "$dir" ]; then
    echo "install_px4_control_params: $dir not found, skipping" >&2
    return 0
  fi
  local ev_delay=""
  local ev_ctrl=""
  if [ "$mode" != "gps" ]; then
    ev_delay="$(_px4_control_number EKF2_EV_DELAY "${EKF2_EV_DELAY:-0}" 0 300 0)" || return 1
    ev_ctrl="$(_px4_control_number EKF2_EV_CTRL "${EKF2_EV_CTRL:-11}" 0 15 1)" || return 1
  fi
  local tmp
  tmp="$(mktemp)"
  {
    echo "# px4_control sim profile. Sourced by gz_start_px4_gz_sim.sh."
    echo "# mode=$mode"
    echo "# NAV_DLL_ACT=0 is sim-only so headless SITL can arm without QGroundControl."
    echo "export PX4_PARAM_COM_OF_LOSS_T=1.0"
    echo "export PX4_PARAM_COM_OBL_RC_ACT=4"
    echo "export PX4_PARAM_COM_RC_LOSS_T=35.0"
    echo "export PX4_PARAM_NAV_RCL_ACT=1"
    echo "export PX4_PARAM_NAV_DLL_ACT=0"
    echo "export PX4_PARAM_UXRCE_DDS_SYNCT=0"
    if [ "$mode" = "gps" ]; then
      echo "export PX4_PARAM_EKF2_EV_CTRL=0"
      echo "export PX4_PARAM_EKF2_HGT_REF=1"
      echo "export PX4_PARAM_EKF2_GPS_CTRL=7"
      echo "export PX4_PARAM_EKF2_GPS_P_NOISE=0.5"
      echo "export PX4_PARAM_EKF2_GPS_V_NOISE=0.3"
    else
      echo "export PX4_PARAM_EKF2_EV_CTRL=${ev_ctrl}"
      echo "export PX4_PARAM_EKF2_MAG_TYPE=5"
      echo "export PX4_PARAM_EKF2_HGT_REF=3"
      echo "export PX4_PARAM_EKF2_EV_DELAY=${ev_delay}"
      echo "export PX4_PARAM_EKF2_EV_NOISE_MD=0"
      echo "export PX4_PARAM_EKF2_GPS_CTRL=0"
      echo "export PX4_PARAM_COM_ARM_WO_GPS=1"
    fi
  } > "$tmp"
  mv "$tmp" "$env_file"
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  install_px4_control_params "$@"
fi
