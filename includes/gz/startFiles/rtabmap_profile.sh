#!/bin/bash
# RTAB-Map profile for VISION_PROFILE. Source this, then call:
#   rtabmap_profile_args stereo|rgbd|rgbd-wrapper
# It sets RTAB_CFG, RTAB_ARGS, and RTAB_ODOM.
#
# RTAB_CFG is startFiles/rtabmap_profiles/<profile>.ini. The launch passes
# it as cfg:=, which both the SLAM node and the odometry node load. Profile
# parameters live in that file, not on the command line.
# RTAB_ARGS is -d plus the camera-mode grid keys. RTAB_ODOM is the RGB-D
# odometry key (Vis/DepthAsMask) when this camera mode uses it.
# cpu stereo also adds Stereo/MaxDisparity 64, which RGB-D does not use.
#
# stderr gets the effective profile, resolution, rate, and every parameter.
# rtabmap_param source=ini lines match the file. source=camera lines are the
# mode overrides. Unset VISION_PROFILE is cpu.
#
# The Rtabmap container bind-mounts this directory at
# /home/px4/volume/startFiles, apart from includes/gz. ../oakd_s2 does not
# exist there. The module is $HOME/volume/includes/gz/oakd_s2, which is the
# includes/gz mount. A repo checkout still has ../oakd_s2. OAKD_S2_DIR wins
# when it is set. A missing module or a failed resolver returns 1. The launch
# scripts exit on that, so RTAB-Map is not started with an empty cfg.

rtabmap_profile_args() {
  local camera="${1:-stereo}"
  export VISION_PROFILE="${VISION_PROFILE:-cpu}"
  local here py quoted relative container
  here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  relative="${here}/../oakd_s2/rtabmap_params.py"
  container="${HOME:-}/volume/includes/gz/oakd_s2/rtabmap_params.py"
  if [ -n "${OAKD_S2_DIR:-}" ]; then
    py="${OAKD_S2_DIR}/rtabmap_params.py"
  elif [ -f "${relative}" ]; then
    py="${relative}"
  elif [ -f "${container}" ]; then
    py="${container}"
  else
    echo "rtabmap_profile_args: missing rtabmap_params.py (tried ${relative} and ${container})" >&2
    return 1
  fi
  if [ ! -f "${py}" ]; then
    echo "rtabmap_profile_args: missing ${py}" >&2
    return 1
  fi
  if ! quoted="$(python3 "${py}" --shell --camera "${camera}")"; then
    echo "rtabmap_profile_args: ${py} failed for camera ${camera}; not launching" >&2
    return 1
  fi
  if [ -z "${quoted}" ]; then
    echo "rtabmap_profile_args: ${py} printed no RTAB_CFG/RTAB_ARGS/RTAB_ODOM" >&2
    return 1
  fi
  # shellcheck disable=SC2086
  eval "${quoted}"
}
