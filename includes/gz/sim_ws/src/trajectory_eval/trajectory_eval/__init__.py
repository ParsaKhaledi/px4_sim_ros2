"""Compare ground truth, GPS, RTAB-Map, and PX4 EKF2 trajectories.

Live and bag modes read ``/ground_truth/odom``, ``/fmu/out/vehicle_gps_position``,
``/rtabmap/odom``, and ``/fmu/out/vehicle_odometry``. Stamps follow the node
clock, which is sim time when ``use_sim_time`` is set. Topic names are CLI flags.
"""

__version__ = "0.1.0"
