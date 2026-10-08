"""Runtime monitors for the PX4 Gazebo simulation.

Publishes ``/sim/real_time_factor``, TF ``world`` to ``spawn`` and
``base_link_gt``, and serves ``/sim/preflight_check``. When ``IMU_SOURCE=px4``
it also publishes ``/imu``. Thresholds are the ``PREFLIGHT_*``, ``IMU_*``,
and ``HEADLESS_SOFTWARE`` variables in ``.env.example``.
"""
