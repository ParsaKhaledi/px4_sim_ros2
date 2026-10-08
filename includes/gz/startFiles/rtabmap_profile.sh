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
# mode overrides. A full or hw stereo size below 1280x720 warns on stderr.
# full or hw with no GPU also warns once and still starts. Unset
# VISION_PROFILE is cpu.

rtabmap_profile_args() {
  local camera="${1:-stereo}"
  export VISION_PROFILE="${VISION_PROFILE:-cpu}"
  local here py quoted
  here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  py="${here}/../oakd_s2/rtabmap_params.py"
  if [ ! -f "${py}" ]; then
    echo "rtabmap_profile_args: missing ${py}" >&2
    return 1
  fi
  quoted="$(python3 "${py}" --shell --camera "${camera}")" || return 1
  # shellcheck disable=SC2086
  eval "${quoted}"
}
