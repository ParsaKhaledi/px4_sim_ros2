#!/bin/bash
# Write the sim-only PX4 parameter profile as PX4_PARAM_* assignments
# and airframe .post scripts that survive the x500 set-default.
#
# PX4 v1.17 rcS applies every PX4_PARAM_<NAME> from the environment
# before the airframe and before ekf2 start:
#   ROMFS/px4fmu_common/init.d-posix/rcS
# Appending "param set" to px4-rc.params does not work. That file is
# not sourced. gz_start_px4_gz_sim.sh sources the env file this writes
# before `make px4_sitl`.
#
# NAV_DLL_ACT's compiled default is already 0, so the early PX4_PARAM_
# set does not mark it configured. 4001_gz_x500 / 4002_gz_x500_depth
# then run `param set-default NAV_DLL_ACT 2`. The .post file is sourced
# after that airframe block and forces 0 so headless SITL can arm
# without QGroundControl.
#
# Usage: install_px4_control_params <env-file> [vision|gps]
#
# Vision-mode environment overrides (see docs/px4_control.md):
#   EKF2_EV_DELAY   milliseconds, 0..300, default 0
#   EKF2_EV_CTRL    integer bitmask 0..15, default 9 (horizontal position and yaw)

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

# Force failsafes after the airframe. Keep this in sync with the env exports.
# When px4_params.py apply already wrote the full sim.params .post, keep it.
_px4_control_write_failsafe_post() {
  local post_file="$1"
  if [ -f "$post_file" ] && grep -q 'param set NAV_DLL_ACT' "$post_file"; then
    return 0
  fi
  cat > "$post_file" << 'EOF'
# Written by install_px4_control_params.bash. Sourced after the airframe.
# NAV_DLL_ACT must be set here: PX4_PARAM_ alone does not stick when the
# firmware default is already 0 and the airframe set-default puts 2 back.
param set NAV_DLL_ACT 0
param set NAV_RCL_ACT 1
EOF
}

_px4_control_install_airframe_posts() {
  local px4_root="$1"
  local written=0
  local airframes
  for airframes in \
      "$px4_root/ROMFS/px4fmu_common/init.d-posix/airframes" \
      "$px4_root/build/px4_sitl_default/etc/init.d-posix/airframes"
  do
    if [ ! -d "$airframes" ]; then
      continue
    fi
    _px4_control_write_failsafe_post "$airframes/4001_gz_x500.post"
    _px4_control_write_failsafe_post "$airframes/4002_gz_x500_depth.post"
    written=1
  done
  return 0
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
    ev_ctrl="$(_px4_control_number EKF2_EV_CTRL "${EKF2_EV_CTRL:-9}" 0 15 1)" || return 1
  fi
  local tmp
  tmp="$(mktemp)"
  {
    echo "# px4_control sim profile. Sourced by gz_start_px4_gz_sim.sh before make."
    echo "# mode=$mode"
    echo "# NAV_DLL_ACT=0 is sim-only so headless SITL can arm without QGroundControl."
    echo "# Airframe .post (same installer) re-applies NAV_DLL_ACT after set-default."
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
      # Vision: EV horizontal + yaw (9), baro height (0), GNSS fusion off (0).
      echo "export PX4_PARAM_EKF2_EV_CTRL=${ev_ctrl}"
      echo "export PX4_PARAM_EKF2_MAG_TYPE=5"
      echo "export PX4_PARAM_EKF2_HGT_REF=0"
      echo "export PX4_PARAM_EKF2_EV_DELAY=${ev_delay}"
      echo "export PX4_PARAM_EKF2_EV_NOISE_MD=0"
      echo "export PX4_PARAM_EKF2_GPS_CTRL=0"
      echo "export PX4_PARAM_EKF2_RNG_CTRL=1"
      echo "export PX4_PARAM_COM_ARM_WO_GPS=1"
      echo "export PX4_PARAM_GF_MAX_VER_DIST=3.0"
      echo "export PX4_PARAM_GF_ACTION=5"
    fi
  } > "$tmp"
  mv "$tmp" "$env_file"

  # Derive PX4-Autopilot root from the env file path when possible.
  local px4_root
  px4_root="$(cd "$dir" && pwd)"
  _px4_control_install_airframe_posts "$px4_root"
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  install_px4_control_params "$@"
fi
