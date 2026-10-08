#!/bin/bash
# Rewrite the px4_control block in a PX4 rc params file.
# Safe to run on every container start: the previous block is removed first,
# so parameters do not stack the way `>>` did in gz_modifications.bash.
#
# Usage: install_px4_control_params <px4-rc.params> [vision|gps]

install_px4_control_params() {
  local rc_file="$1"
  local mode="${2:-${ESTIMATION_MODE:-vision}}"
  local begin="# BEGIN px4_control"
  local end="# END px4_control"
  mode="$(printf '%s' "$mode" | tr '[:upper:]' '[:lower:]')"
  if [ "$mode" != "vision" ] && [ "$mode" != "gps" ]; then
    echo "install_px4_control_params: ESTIMATION_MODE must be vision or gps, got '$mode'" >&2
    return 1
  fi
  local dir
  dir="$(dirname "$rc_file")"
  if [ ! -d "$dir" ]; then
    echo "install_px4_control_params: $dir not found, skipping" >&2
    return 0
  fi
  # PX4 v1.17 does not source px4-rc.params. The old gz_modifications.bash
  # append created a file the startup script never read. Create the file
  # and hook rcS so the block runs after the airframe and before ekf2.
  local tmp
  tmp="$(mktemp)"
  if [ -f "$rc_file" ]; then
    awk -v begin="$begin" -v end="$end" '
      $0 == begin { skip = 1; next }
      $0 == end { skip = 0; next }
      skip { next }
      $0 == "param set-default COM_RC_LOSS_T 35.0" { next }
      $0 == "param set-default NAV_RCL_ACT 1" { next }
      { print }
    ' "$rc_file" > "$tmp"
  else
    : > "$tmp"
  fi
  {
    echo "$begin"
    echo "# Applied by includes/gz/params/install_px4_control_params.bash"
    echo "# mode=$mode"
    echo "param set COM_OF_LOSS_T 1.0"
    echo "param set COM_OBL_RC_ACT 4"
    echo "param set COM_RC_LOSS_T 35.0"
    echo "param set NAV_RCL_ACT 1"
    if [ "$mode" = "gps" ]; then
      echo "param set EKF2_EV_CTRL 0"
      echo "param set EKF2_HGT_REF 1"
      echo "param set EKF2_GPS_CTRL 7"
      echo "param set EKF2_GPS_P_NOISE 0.5"
      echo "param set EKF2_GPS_V_NOISE 0.3"
    else
      # Vision is the primary aid. GPS stays enabled at a lower weight:
      # lon/lat + velocity (bits 0 and 2 => 5), not altitude, so height
      # follows vision. The simulated GPS sensor is not removed.
      echo "param set EKF2_EV_CTRL 15"
      echo "param set EKF2_HGT_REF 3"
      echo "param set EKF2_EV_DELAY 50.0"
      echo "param set EKF2_EV_NOISE_MD 0"
      echo "param set EKF2_GPS_CTRL 5"
      echo "param set EKF2_GPS_P_NOISE 5.0"
      echo "param set EKF2_GPS_V_NOISE 1.0"
      echo "param set COM_ARM_WO_GPS 1"
    fi
    echo "$end"
  } >> "$tmp"
  mv "$tmp" "$rc_file"
  _px4_control_hook_rcs "$dir/rcS"
  _px4_control_hook_cmake "$dir/CMakeLists.txt"
}

# Insert one source of px4-rc.params immediately before EKF2 starts.
_px4_control_hook_rcs() {
  local rcs="$1"
  if [ ! -f "$rcs" ]; then
    return 0
  fi
  if grep -q '# BEGIN px4_control source' "$rcs"; then
    return 0
  fi
  local tmp
  tmp="$(mktemp)"
  awk '
    $0 == "# state estimator selection" && !inserted {
      print "# BEGIN px4_control source"
      print "if [ -f ${R}etc/init.d-posix/px4-rc.params ]"
      print "then"
      print "	. ${R}etc/init.d-posix/px4-rc.params"
      print "fi"
      print "# END px4_control source"
      inserted = 1
    }
    { print }
  ' "$rcs" > "$tmp"
  mv "$tmp" "$rcs"
}

# px4_add_romfs_files() only copies names listed here. Without this entry
# the params file never reaches the SITL rootfs.
_px4_control_hook_cmake() {
  local cmake="$1"
  if [ ! -f "$cmake" ]; then
    return 0
  fi
  if grep -q 'px4-rc.params' "$cmake"; then
    return 0
  fi
  local tmp
  tmp="$(mktemp)"
  awk '
    $0 == "\trcS" && !inserted {
      print "\tpx4-rc.params"
      inserted = 1
    }
    { print }
  ' "$cmake" > "$tmp"
  mv "$tmp" "$cmake"
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  install_px4_control_params "$@"
fi
