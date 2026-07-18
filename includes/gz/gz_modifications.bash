#!/bin/bash

echo "Start Modifications for running Simulation"
input=$1
WORKDIR=/home/px4
# Set custom params:
echo "param set-default COM_RC_LOSS_T 35.0" >> $WORKDIR/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params
echo "param set-default NAV_RCL_ACT 1" >> $WORKDIR/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params
# Offboard: ignore RC-loss failsafe in OFFBOARD (bit 2); give proof-of-life more
# margin when Gazebo runs slower than realtime (COM_OF_LOSS_T uses msg age).
echo "param set-default COM_RCL_EXCEPT 4" >> $WORKDIR/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params
echo "param set-default COM_OF_LOSS_T 5.0" >> $WORKDIR/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params
# If failsafe drops to POSCTL after "takeoff detected", hold near mission height
# instead of the default ~0.5–2.5 m hover that looked like a takeoff ceiling.
echo "param set-default MIS_TAKEOFF_ALT 1.0" >> $WORKDIR/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params
# Gazebo-clock scenario (PX4 ROS 2 user guide): let Gazebo /clock be the single
# time source for PX4 + ROS 2 instead of PX4's own agent-wall-clock sync, which
# is only valid at real-time-factor ~= 1 and conflicts with use_sim_time=true.
# Inbound /fmu/in messages MUST use timestamp=0 so uXRCE rewrites them with
# hrt_absolute_time(); ROS sim-time stamps lag PX4 and trip offboard-loss.
echo "param set-default UXRCE_DDS_SYNCT 0" >> $WORKDIR/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params
# Indoor VIO flight: x500_depth's default EKF2_HGT_REF=1 (GPS) fuses the
# SITL GPS plugin's ALTITUDE into the height estimate alongside our
# RTAB-Map vision odometry. That simulated GPS altitude is very noisy
# indoors (observed >1m jumps between consecutive samples) and fights the
# vision fix, making the fused Z estimate diverge from truth. Make vision
# the EKF height reference and drop GPS altitude fusion (bit 1), but KEEP
# GPS lon/lat + 3D velocity fusion (bits 0+2 = 5) — that data was smooth in
# testing and acts as a horizontal position/velocity cross-check so the
# estimator doesn't free-drift on IMU+vision alone if vision is briefly
# lost/rejected as an outlier.
echo "param set-default EKF2_GPS_CTRL 5" >> $WORKDIR/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params
echo "param set-default EKF2_HGT_REF 3" >> $WORKDIR/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params
# Camera Modifications:
echo "Selected Camera Type: $input"

if [ "$input" = Stereo ] || [ "$input" = stereo ]; then
    rm -rf $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    cp -rv $WORKDIR/volume/includes/gz/models/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/
    cp -rv $WORKDIR/volume/includes/gz/worlds/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/worlds/
    mv -v  $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite-stereo $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    echo "Replacements with $input camera is done"
elif [ "$input" = rgbd ] || [ "$input" = RGBD ] ; then
    rm -rf $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    cp -rv $WORKDIR/volume/includes/gz/models/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/
    cp -rv $WORKDIR/volume/includes/gz/worlds/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/worlds/
    mv -v  $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite-rgbd $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    echo "Replacements with $input camera is done"
else
    echo "Invalid input, please try again."
    exit 1
fi
