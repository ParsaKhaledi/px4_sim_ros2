#!/bin/bash
# RTAB-Map args for VISION_PROFILE. Source this, then call:
#   rtabmap_profile_args stereo|rgbd|rgbd-wrapper
# It sets RTAB_ARGS and RTAB_ODOM.
#
# Every key is a real Parameters.h entry (rtabmap master, and the same names
# in the 0.21-era header). parseArguments drops a key that is not in that map
# and does not warn. These are passed inside args:= / odom_args:=, not as
# launch arguments.

rtabmap_profile_args() {
  local camera="${1:-stereo}"
  local profile
  profile="$(printf '%s' "${VISION_PROFILE:-full}" | tr '[:upper:]' '[:lower:]')"
  local ground obstacle extra_grid extra_odom stereo_disp
  extra_grid=""
  extra_odom=""
  stereo_disp=""
  case "${camera}" in
    stereo)
      ground=1.0
      obstacle=2.0
      ;;
    rgbd)
      ground=0.5
      obstacle=2.2
      extra_odom=" --Vis/DepthAsMask true"
      ;;
    rgbd-wrapper)
      ground=0.5
      obstacle=2.2
      extra_grid=" --Grid/RayTracing true --Grid/3D true --Grid/FlatObstacleDetected true"
      extra_odom=" --Vis/DepthAsMask true"
      ;;
    *)
      echo "rtabmap_profile_args: expected stereo, rgbd, or rgbd-wrapper" >&2
      return 1
      ;;
  esac
  if [ "${profile}" = "cpu" ]; then
    # Stereo/MaxDisparity is pixels. Half-resolution fx halves the disparity of
    # a given depth, so 64 matches a full-resolution cap of 128. RGB-D does
    # not compute that disparity.
    if [ "${camera}" = "stereo" ]; then
      stereo_disp=" --Stereo/MaxDisparity 64"
    fi
    RTAB_ARGS="-d --Optimizer/GravitySigma 0.1 --Vis/FeatureType 10 --Kp/DetectorStrategy 10 --Vis/MaxFeatures 400 --Vis/MinInliers 15 --Kp/MaxFeatures 300 --Grid/MapFrameProjection true --Grid/NormalsSegmentation false --Grid/MaxGroundHeight ${ground} --Grid/MaxObstacleHeight ${obstacle} --Grid/CellSize 0.1 --Grid/RangeMax 5 --Rtabmap/DetectionRate 1 --RGBD/StartAtOrigin true${extra_grid}${stereo_disp}"
    # VisKeyFrameThr default 150 is above typical inliers, so every frame is a
    # keyframe and frame time grows. 30 is not yet measured live.
    RTAB_ODOM="--Odom/Strategy 0 --Odom/ResetCountdown 1 --Odom/VisKeyFrameThr 30 --OdomF2M/MaxSize 1000 --Vis/CorGuessWinSize 20 --Vis/EstimationType 1${extra_odom}"
  else
    RTAB_ARGS="-d --Optimizer/GravitySigma 0.1 --Vis/FeatureType 10 --Kp/DetectorStrategy 10 --Vis/MaxFeatures 1000 --Vis/MinInliers 20 --Grid/MapFrameProjection true --Grid/NormalsSegmentation false --Grid/MaxGroundHeight ${ground} --Grid/MaxObstacleHeight ${obstacle} --RGBD/StartAtOrigin true${extra_grid}"
    RTAB_ODOM="--Odom/Strategy 0 --Odom/ResetCountdown 1 --OdomF2M/MaxSize 2000 --Vis/CorGuessWinSize 40 --Vis/EstimationType 1${extra_odom}"
  fi
}
