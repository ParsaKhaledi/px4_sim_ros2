# Graph Report - px4_sim_ros2  (2026-11-13)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 3393 nodes · 6587 edges · 197 communities (151 shown, 46 thin omitted)
- Extraction: 96% EXTRACTED · 4% INFERRED · 0% AMBIGUOUS · INFERRED: 250 edges (avg confidence: 0.84)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `1529f396`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Community 0
- Community 1
- Community 2
- Community 3
- Community 4
- Community 5
- Community 6
- Community 7
- Community 8
- Community 9
- Community 10
- Community 11
- Community 12
- Community 13
- Community 14
- Community 15
- Community 16
- Community 17
- Community 18
- Community 19
- Community 20
- Community 21
- Community 22
- Community 23
- Community 24
- Community 25
- Community 26
- Community 27
- Community 28
- Community 29
- Community 30
- Community 31
- Community 32
- Community 33
- Community 34
- Community 35
- Community 36
- Community 37
- Community 38
- Community 39
- Community 40
- Community 41
- Community 42
- Community 43
- Community 44
- Community 45
- Community 46
- Community 47
- Community 48
- Community 49
- Community 50
- Community 51
- Community 52
- Community 53
- Community 54
- Community 55
- Community 56
- Community 57
- Community 58
- Community 59
- Community 60
- Community 61
- Community 62
- Community 63
- Community 64
- Community 65
- Community 66
- Community 67
- Community 68
- Community 69
- Community 70
- Community 71
- Community 72
- Community 73
- Community 74
- Community 75
- Community 76
- Community 77
- Community 78
- Community 79
- Community 80
- Community 81
- Community 82
- Community 83
- Community 84
- Community 85
- Community 86
- Community 87
- Community 88
- Community 89
- Community 90
- Community 91
- Community 92
- Community 93
- Community 94
- Community 95
- Community 97
- Community 98
- Community 99
- Community 100
- Community 101
- Community 102
- Community 103
- Community 104
- Community 105
- Community 106
- Community 107
- Community 108
- Community 109
- Community 110
- Community 111
- Community 112
- Community 113
- Community 114
- Community 115
- Community 116
- Community 117
- Community 118
- Community 119
- Community 120
- Community 121
- Community 122
- Community 123
- Community 124
- Community 125
- Community 126
- Community 127
- Community 128
- Community 129
- Community 130
- Community 131
- Community 132
- Community 133
- Community 134
- Community 135
- Community 136
- Community 137
- Community 138
- Community 139
- Community 140
- Community 141
- Community 142
- Community 144
- Community 145
- Community 146
- Community 147
- Community 148
- Community 149
- Community 150
- Community 151
- Community 152
- Community 153
- Community 154
- Community 155
- Community 156
- Community 157
- Community 159
- Community 160
- Community 161
- Community 162
- Community 163
- Community 164
- Community 165
- Community 166
- Community 167
- Community 168
- Community 169
- Community 170
- Community 171
- Community 172
- Community 173
- Community 174
- Community 176
- Community 180
- Community 181
- Community 182
- Community 183
- Community 184
- Community 185
- Community 186
- Community 187
- Community 188
- Community 193
- Community 194
- Community 195

## God Nodes (most connected - your core abstractions)
1. `Px4ControlNode` - 68 edges
2. `MotionExecutive` - 57 edges
3. `FlightWatch` - 51 edges
4. `Snapshot` - 41 edges
5. `FollowPath` - 40 edges
6. `FollowPath` - 39 edges
7. `FollowPath` - 39 edges
8. `FollowPath` - 39 edges
9. `ros__parameters` - 39 edges
10. `ros__parameters` - 39 edges

## Surprising Connections (you probably didn't know these)
- `test_flight_mechanics_defaults()` --calls--> `load_thresholds()`  [INFERRED]
  tests/test_grading.py → tests/e2e/grading.py
- `test_headless_hover()` --references--> `flight`  [EXTRACTED]
  tests/test_hover_flight.py → .github/workflows/ci.yml
- `DockerBuild.sh script` --calls--> `load_repo_env()`  [EXTRACTED]
  DockerBuild.sh → scripts/load_env.sh
- `Px4ControlNode` --uses--> `HeightGuard`  [INFERRED]
  ros2_ws/src/px4_control/px4_control/node.py → ros2_ws/src/px4_control/px4_control/e2e_height.py
- `Px4ControlNode` --uses--> `LidarHeightGuard`  [INFERRED]
  ros2_ws/src/px4_control/px4_control/node.py → ros2_ws/src/px4_control/px4_control/lidar_height.py

## Import Cycles
- None detected.

## Communities (197 total, 46 thin omitted)

### Community 0 - "Community 0"
Cohesion: 0.06
Nodes (73): resources, capabilities, count, driver, CAM_COLOR_HEIGHT, CAM_COLOR_WIDTH, CAM_PITCH_DEG, CAM_RATE_HZ (+65 more)

### Community 1 - "Community 1"
Cohesion: 0.07
Nodes (6): ArmDecision, _fill3(), Px4ControlNode, feedback(), feedback(), feedback()

### Community 2 - "Community 2"
Cohesion: 0.07
Nodes (34): attitude_from_enu_quat(), body_horizontal_to_ned(), body_offset_to_enu(), command_age(), covariance_block(), diagonal_variances(), enu_flu_to_ned_frd_rotation(), enu_to_ned() (+26 more)

### Community 3 - "Community 3"
Cohesion: 0.08
Nodes (57): resources, CAM_COLOR_HEIGHT, CAM_COLOR_WIDTH, CAM_PITCH_DEG, CAM_RATE_HZ, CAM_STEREO_HEIGHT, CAM_STEREO_WIDTH, CAM_X (+49 more)

### Community 4 - "Community 4"
Cohesion: 0.07
Nodes (26): heartbeat(), _ident(), param_request_read(), param_set(), ParamValue, parse_frames(), _parse_param_value(), read_params() (+18 more)

### Community 5 - "Community 5"
Cohesion: 0.07
Nodes (27): build_parser(), _env_float(), fail_on_loss_enabled(), inlier_gate_value(), interval_rate(), json_number(), load_env_file(), load_repo_env() (+19 more)

### Community 6 - "Community 6"
Cohesion: 0.07
Nodes (11): camera_overrides(), effective_parameters(), ini_path(), launch_strings(), load_profile_ini(), main(), parse_rtabmap_ini(), _rate_text() (+3 more)

### Community 7 - "Community 7"
Cohesion: 0.12
Nodes (25): _line(), test_align_se3_recovers_rigid_transform_without_scale(), test_constant_offset_is_removed_by_takeoff_frame_and_by_alignment(), test_gps_noise_on_a_biased_cloud(), test_lateral_drift_rpe_and_final_error(), test_resample_pairs_different_rates(), test_yaw_error_detects_a_growing_heading(), wrap_pi() (+17 more)

### Community 8 - "Community 8"
Cohesion: 0.11
Nodes (36): _along(), _check(), grade_hover(), _grade_hover_checks(), _grade_hover_clock(), _grade_leg(), grade_mission(), _grade_yaw() (+28 more)

### Community 9 - "Community 9"
Cohesion: 0.09
Nodes (35): main(), mismatches(), parse_param_value(), apply(), clear_parameter_store(), default_params_dir(), ensure_post_before_ekf2(), env_overrides() (+27 more)

### Community 10 - "Community 10"
Cohesion: 0.10
Nodes (6): FlightCrash, FlightWatch, down(), at_height(), held(), inside()

### Community 11 - "Community 11"
Cohesion: 0.08
Nodes (27): close_vec(), main(), check_fx(), check_image(), wait_for(), quat_angle(), record(), format_summary() (+19 more)

### Community 12 - "Community 12"
Cohesion: 0.25
Nodes (42): CAM_COLOR_HEIGHT, CAM_COLOR_WIDTH, CAM_PITCH_DEG, CAM_RATE_HZ, CAM_STEREO_HEIGHT, CAM_STEREO_WIDTH, CAM_X, CAM_Y (+34 more)

### Community 13 - "Community 13"
Cohesion: 0.09
Nodes (20): candidate_base_models(), candidate_server_configs(), candidate_worlds(), ensure_model_systems(), main(), patch_model(), plugin_block(), removal_line() (+12 more)

### Community 14 - "Community 14"
Cohesion: 0.10
Nodes (26): check_max(), check_min(), distance_sensor_result(), distance_sensor_topic(), header_stamp_s(), lidar_down_enabled(), line(), match_versioned_topic() (+18 more)

### Community 15 - "Community 15"
Cohesion: 0.11
Nodes (19): _finite_vector(), _invalid_diagonal(), _is_null_pose(), lost_covariance(), OdomSample, VisionOdometryBridge, VisualOdom, _Msg (+11 more)

### Community 16 - "Community 16"
Cohesion: 0.05
Nodes (40): acc_lim_theta, acc_lim_x, acc_lim_y, angular_granularity, BaseObstacle.scale, BaseObstacle.sum_scores, critics, debug_trajectory_details (+32 more)

### Community 17 - "Community 17"
Cohesion: 0.09
Nodes (20): apply_origin_to_worlds(), ecef_to_geodetic(), enu_to_geodetic(), geodetic_to_ecef(), main(), origin_from_env(), parse_spawn_xyz(), px4_world_dirs() (+12 more)

### Community 18 - "Community 18"
Cohesion: 0.05
Nodes (39): acc_lim_theta, acc_lim_x, acc_lim_y, angular_granularity, BaseObstacle.scale, critics, debug_trajectory_details, decel_lim_theta (+31 more)

### Community 19 - "Community 19"
Cohesion: 0.05
Nodes (39): acc_lim_theta, acc_lim_x, acc_lim_y, angular_granularity, BaseObstacle.scale, critics, debug_trajectory_details, decel_lim_theta (+31 more)

### Community 20 - "Community 20"
Cohesion: 0.05
Nodes (39): acc_lim_theta, acc_lim_x, acc_lim_y, angular_granularity, BaseObstacle.scale, critics, debug_trajectory_details, decel_lim_theta (+31 more)

### Community 21 - "Community 21"
Cohesion: 0.10
Nodes (19): world_enu_point_to_local(), height_above_floor(), lidar_height_log(), LidarHeightGuard, reading_in_range(), tilt_compensated_height(), _feed(), test_a_sample_back_inside_the_tolerance_resets_the_timer() (+11 more)

### Community 22 - "Community 22"
Cohesion: 0.05
Nodes (37): x-ros-env, CAM_COLOR_HEIGHT, CAM_COLOR_WIDTH, CAM_PITCH_DEG, CAM_RATE_HZ, CAM_STEREO_HEIGHT, CAM_STEREO_WIDTH, CAM_X (+29 more)

### Community 23 - "Community 23"
Cohesion: 0.05
Nodes (37): ros__parameters, alpha1, alpha2, alpha3, alpha4, alpha5, base_frame_id, beam_skip_distance (+29 more)

### Community 24 - "Community 24"
Cohesion: 0.05
Nodes (37): ros__parameters, alpha1, alpha2, alpha3, alpha4, alpha5, base_frame_id, beam_skip_distance (+29 more)

### Community 25 - "Community 25"
Cohesion: 0.05
Nodes (37): ros__parameters, alpha1, alpha2, alpha3, alpha4, alpha5, base_frame_id, beam_skip_distance (+29 more)

### Community 26 - "Community 26"
Cohesion: 0.10
Nodes (19): candidate_models(), covariance_topic_problems(), _ensure_covariance(), main(), patch_file(), patch_text(), _plugin_blocks(), px4_x500_model_sdfs() (+11 more)

### Community 27 - "Community 27"
Cohesion: 0.11
Nodes (18): flu_enu_quaternion(), flu_enu_quaternion_xyzw(), frd_to_flu(), gravity_quaternion_xyzw(), quat_from_rpy(), quat_mul(), quat_normalize(), quat_to_rot() (+10 more)

### Community 28 - "Community 28"
Cohesion: 0.08
Nodes (11): ModeCommand, OffboardStreamGate, parse_mode(), main(), _px4_qos(), estimation_mode_from_env(), normalize_estimation_mode(), yaw_rate_from_env() (+3 more)

### Community 29 - "Community 29"
Cohesion: 0.06
Nodes (33): px4_control, ros__parameters, a_brake, arm_timeout_s, cmd_vel_timeout_s, cmd_vel_topic, e2e_height_tolerance_m, estimation_mode (+25 more)

### Community 30 - "Community 30"
Cohesion: 0.12
Nodes (16): test_geodetic_equator_offsets(), test_level_body_frd_aligned_with_ned_points_north_in_enu(), test_ned_position_to_enu(), test_yaw_only_ned_to_enu(), ecef_to_enu(), geodetic_to_ecef(), geodetic_to_enu(), ned_frd_to_enu_flu() (+8 more)

### Community 31 - "Community 31"
Cohesion: 0.09
Nodes (19): yaw_from_quat(), _block_value(), gazebo_env(), match_score(), ned_from_enu(), _one_pose(), parse_pose_v(), parse_rtf() (+11 more)

### Community 32 - "Community 32"
Cohesion: 0.12
Nodes (17): download_fuel_file(), _download_named(), _ensure_model_config(), _follow_local_sidecars(), _follow_mesh_sidecars(), fuel_model_version(), _get(), models_dir() (+9 more)

### Community 33 - "Community 33"
Cohesion: 0.11
Nodes (18): _add(), _allowed_stereo_text(), _env_int(), gpu_visible(), matmul(), matvec(), _nvidia_smi_ok(), optical_origin_in_base() (+10 more)

### Community 34 - "Community 34"
Cohesion: 0.10
Nodes (13): _color_full(), color_intrinsics(), corrected_projection(), diagonal_fov_deg(), hfov_from_intrinsics(), rectified_k(), rectified_p(), scale_intrinsics() (+5 more)

### Community 35 - "Community 35"
Cohesion: 0.07
Nodes (30): cleanup(), E2E_ATTEMPT_TIMEOUT, E2E_CAMERA_MIN_VARIANCE, E2E_CHECK_CAMERAS, E2E_CRASH_DIVERGENCE_M, E2E_CRASH_IMPACT_SPEED_MPS, E2E_CRASH_MIN_HEIGHT_M, E2E_CRASH_TILT_DEG (+22 more)

### Community 36 - "Community 36"
Cohesion: 0.11
Nodes (12): height_abort_log(), HeightGuard, grade_hover(), HoverGrade, Limits, _Settle, _limits(), _snap() (+4 more)

### Community 37 - "Community 37"
Cohesion: 0.11
Nodes (11): check_rate(), gated_rate_hz(), RateTracker, _span_hz(), stale_limit_s(), topic_rate_line(), _constant_imu(), test_full_profile_fails_a_100_hz_sim_imu() (+3 more)

### Community 38 - "Community 38"
Cohesion: 0.17
Nodes (12): box_inertia(), Mount, sensor_layouts(), _housing(), render_sdf(), render_urdf(), child_text(), findall() (+4 more)

### Community 39 - "Community 39"
Cohesion: 0.08
Nodes (27): amcl, behavior_server, bt_navigator, bt_navigator_navigate_through_poses_rclcpp_node, ros__parameters, bt_navigator_navigate_to_pose_rclcpp_node, ros__parameters, controller_server (+19 more)

### Community 40 - "Community 40"
Cohesion: 0.28
Nodes (19): MotionExecutive, Setpoint, _limits(), _snap(), test_abort_and_land_preempts_takeoff(), test_abort_while_moving_brakes_along_the_path(), test_cmd_vel_position_is_finite_and_then_brakes(), test_cmd_vel_timeout_uses_the_twist_stamp_once() (+11 more)

### Community 41 - "Community 41"
Cohesion: 0.12
Nodes (11): Phase, BrakeSample, BrakeThenHold, _clamp(), ScalarStep, step_scalar(), step_vector2(), step_yaw() (+3 more)

### Community 42 - "Community 42"
Cohesion: 0.10
Nodes (27): bt_navigator, ros__parameters, ros__parameters, cost_scaling_factor, inflation_radius, plugin, ros__parameters, always_send_full_costmap (+19 more)

### Community 43 - "Community 43"
Cohesion: 0.15
Nodes (19): test_gps_field_layouts(), _odom(), read_bag(), gps_fields_to_lla(), quat_xyzw_to_wxyz(), _odom_sample(), record_live(), _capture_spawn() (+11 more)

### Community 44 - "Community 44"
Cohesion: 0.12
Nodes (14): CameraTracker, _channel_samples(), is_blank(), pixel_stats(), rtf_summary(), error_stats(), _threshold_dump(), wrap_pi() (+6 more)

### Community 45 - "Community 45"
Cohesion: 0.13
Nodes (14): parse_args(), str2bool(), ensure_distance_sensor(), main(), _is_camera(), main(), tune_cameras(), replace() (+6 more)

### Community 46 - "Community 46"
Cohesion: 0.11
Nodes (25): DISPLAY, GALLIUM_DRIVER, HEADLESS, LIBGL_ALWAYS_SOFTWARE, QT_QPA_PLATFORM, RTABMAPVIZ, XAUTHORITY, environment (+17 more)

### Community 47 - "Community 47"
Cohesion: 0.11
Nodes (26): ros__parameters, ros__parameters, cost_scaling_factor, inflation_radius, plugin, ros__parameters, always_send_full_costmap, bt_loop_duration (+18 more)

### Community 48 - "Community 48"
Cohesion: 0.11
Nodes (26): ros__parameters, ros__parameters, cost_scaling_factor, inflation_radius, plugin, ros__parameters, always_send_full_costmap, bt_loop_duration (+18 more)

### Community 49 - "Community 49"
Cohesion: 0.11
Nodes (26): ros__parameters, ros__parameters, cost_scaling_factor, inflation_radius, plugin, ros__parameters, always_send_full_costmap, bt_loop_duration (+18 more)

### Community 50 - "Community 50"
Cohesion: 0.09
Nodes (24): amcl, behavior_server, bt_navigator_navigate_through_poses_rclcpp_node, ros__parameters, bt_navigator_navigate_to_pose_rclcpp_node, ros__parameters, controller_server, global_costmap (+16 more)

### Community 51 - "Community 51"
Cohesion: 0.09
Nodes (24): behavior_server, bt_navigator, bt_navigator_navigate_through_poses_rclcpp_node, ros__parameters, bt_navigator_navigate_to_pose_rclcpp_node, ros__parameters, controller_server, global_costmap (+16 more)

### Community 52 - "Community 52"
Cohesion: 0.09
Nodes (24): amcl, behavior_server, bt_navigator, bt_navigator_navigate_through_poses_rclcpp_node, ros__parameters, bt_navigator_navigate_to_pose_rclcpp_node, ros__parameters, global_costmap (+16 more)

### Community 53 - "Community 53"
Cohesion: 0.14
Nodes (13): format_rate(), _has_lidar(), lidar_enabled(), lidar_fragment(), lidar_rate_hz(), main(), patch_file(), patch_text() (+5 more)

### Community 54 - "Community 54"
Cohesion: 0.13
Nodes (15): ground_truth_leak_from_info(), gz_topic_publishers(), px4_vision_covariance_topic(), px4_vision_model_name(), summarize(), camera_info_topic(), check_ground_truth_vision_leak(), default_camera_topic() (+7 more)

### Community 56 - "Community 56"
Cohesion: 0.15
Nodes (18): accel_stddev_m_s2(), gyro_stddev_rad_s(), pose_text(), _axis_noise(), _block_span(), _camera_sensor(), _depth_sensor(), _fixed_joint() (+10 more)

### Community 57 - "Community 57"
Cohesion: 0.12
Nodes (12): bridge_gazebo_imu(), configured_imu_publishers(), diagonal_covariance(), imu_source(), imu_stamp_ns(), measure_px4_offset_ns(), normalize_imu_source(), normalize_stamp_mode() (+4 more)

### Community 58 - "Community 58"
Cohesion: 0.19
Nodes (3): GoalStatus, Snapshot, _fly()

### Community 59 - "Community 59"
Cohesion: 0.09
Nodes (22): discovery_s, gz_rtf, enabled, log_name, processes, name, pattern, service (+14 more)

### Community 60 - "Community 60"
Cohesion: 0.16
Nodes (12): load_thresholds(), FlightSetupError, run_px4_mission(), _call(), driver_name(), _dump(), import_drone(), main() (+4 more)

### Community 61 - "Community 61"
Cohesion: 0.16
Nodes (11): default_min_rtf(), _env_float(), expected_imu_hz(), expected_sensor_hz(), _load_geometry(), minimum_rate_hz(), _oak_model_imu_hz(), _profile_imu_hz() (+3 more)

### Community 62 - "Community 62"
Cohesion: 0.11
Nodes (22): enabled, observation_sources, plugin, pointcloud, clearing, data_type, marking, max_obstacle_height (+14 more)

### Community 63 - "Community 63"
Cohesion: 0.12
Nodes (12): consume_stats_line(), find_stats_topic(), iter_real_time_factors(), main(), _ensure_stream(), __init__(), _read(), parse_stats_message() (+4 more)

### Community 64 - "Community 64"
Cohesion: 0.14
Nodes (6): parse_spawn_pose(), quat_from_rpy(), main(), __init__(), spawn_ground_pose(), test_spawn_pose_default()

### Community 65 - "Community 65"
Cohesion: 0.22
Nodes (6): HealthRunner, import_message(), process_alive(), qos_profile(), topic_matches_env(), utc_now()

### Community 66 - "Community 66"
Cohesion: 0.10
Nodes (20): plugin, plugin, ros__parameters, plugin, assisted_teleop, backup, behavior_plugins, costmap_topic (+12 more)

### Community 67 - "Community 67"
Cohesion: 0.10
Nodes (20): plugin, plugin, ros__parameters, plugin, assisted_teleop, backup, behavior_plugins, costmap_topic (+12 more)

### Community 68 - "Community 68"
Cohesion: 0.10
Nodes (20): plugin, plugin, ros__parameters, plugin, assisted_teleop, backup, behavior_plugins, costmap_topic (+12 more)

### Community 69 - "Community 69"
Cohesion: 0.10
Nodes (20): plugin, plugin, ros__parameters, plugin, assisted_teleop, backup, behavior_plugins, costmap_topic (+12 more)

### Community 70 - "Community 70"
Cohesion: 0.21
Nodes (14): crash_reason(), CrashThresholds, load_crash_thresholds(), should_retry(), _obs(), test_healthy_hover_is_not_a_crash(), test_land_phase_may_be_low_and_disarmed(), test_low_altitude_and_impact_while_airborne() (+6 more)

### Community 71 - "Community 71"
Cohesion: 0.11
Nodes (18): runs-on, timeout-minutes, concurrency, cancel-in-progress, group, needs, runs-on, timeout-minutes (+10 more)

### Community 72 - "Community 72"
Cohesion: 0.17
Nodes (10): _env_float(), mount_from_env(), no_gpu_warning(), profile_from_env(), sub_hd_warning(), VisionProfile, gazebo_startup_warnings(), main() (+2 more)

### Community 73 - "Community 73"
Cohesion: 0.18
Nodes (8): test_offset_is_the_lowest_gap_after_fifty_samples(), test_short_recording_leaves_the_offset_empty(), test_written_file_has_only_the_agreed_fields(), test_yaw_from_the_spawn_quaternion(), _xyzw(), build_spawn_document(), write_spawn_json(), yaw_from_xyzw()

### Community 75 - "Community 75"
Cohesion: 0.18
Nodes (12): choose_topic(), topic_version(), test_absent_optional_topic_skips(), test_auto_detect_falls_back_to_existing_topic(), test_auto_detect_prefers_publisher(), test_auto_detect_prefers_version_when_neither_has_a_publisher(), test_camera_env_filter(), test_check_files_cover_services() (+4 more)

### Community 76 - "Community 76"
Cohesion: 0.11
Nodes (18): ros__parameters, plugin, stateful, xy_goal_tolerance, yaw_goal_tolerance, movement_time_allowance, plugin, required_movement_radius (+10 more)

### Community 77 - "Community 77"
Cohesion: 0.14
Nodes (18): pointcloud, clearing, data_type, marking, max_obstacle_height, min_obstacle_height, topic, voxel_layer (+10 more)

### Community 78 - "Community 78"
Cohesion: 0.18
Nodes (10): test_nav2_frames_match_rtabmap_base_link(), _command_text(), _config(), _config_env(), test_camera_env_follows_the_shell(), test_ci_override_is_headless(), test_cpu_and_gpu_pass_the_same_camera_env(), test_every_compose_file_parses() (+2 more)

### Community 79 - "Community 79"
Cohesion: 0.14
Nodes (7): default_tf_pairs(), _env_float(), jumped_to_identity(), __init__(), _norm(), PoseOrigins, test_pose_error_uses_ground_truth_when_rtabmap_starts()

### Community 80 - "Community 80"
Cohesion: 0.12
Nodes (10): ack_failure_text(), combine_failure(), failsafe_reasons(), _magnetometer_arming_text(), prearm_block_reason(), sim_preflight_decision(), test_prearm_names_a_missing_or_failed_magnetometer(), test_prearm_reports_the_flag_and_ignores_healthy_vehicles() (+2 more)

### Community 81 - "Community 81"
Cohesion: 0.12
Nodes (17): CameraType, COMPOSE_FILES, COMPOSE_PROFILES, COMPOSE_PROJECT_NAME, COMPOSE_SERVICES, DISPLAY, GH_CACHE_TOKEN, GZ_HEADLESS_RENDERING (+9 more)

### Community 82 - "Community 82"
Cohesion: 0.18
Nodes (6): _add_topic_args(), _build_parser(), main(), _print_summary(), read_tum(), write_tum()

### Community 83 - "Community 83"
Cohesion: 0.18
Nodes (8): read_gz_rtf(), _run(), test_script_dir_parent_wins_when_the_tree_is_there(), test_sim_gz_dir_override_and_missing_tree(), _install(), test_installer_writes_airframe_post_for_nav_dll(), test_param_env_is_idempotent_and_selects_mode(), test_vision_delay_and_ctrl_come_from_the_environment()

### Community 84 - "Community 84"
Cohesion: 0.12
Nodes (15): discovery_s, log_name, nodes, name, optional, service, topics, group (+7 more)

### Community 85 - "Community 85"
Cohesion: 0.16
Nodes (12): gz_resolve_dir(), gz_resolve_dir.sh script, GZ_IP, GZ_SIM_RESOURCE_PATH, PATH, PX4_GZ_MODEL_POSE, PX4_GZ_WORLD, PX4_SIM_MODEL (+4 more)

### Community 86 - "Community 86"
Cohesion: 0.19
Nodes (3): wrap_pi(), _zeros(), step_yaw_remaining()

### Community 87 - "Community 87"
Cohesion: 0.12
Nodes (15): min-spaces-from-content, extends, ignore, indent-sequences, spaces, max, rules, braces (+7 more)

### Community 88 - "Community 88"
Cohesion: 0.13
Nodes (15): networks, px4, aliases, driver, networks, networks, command, hostname (+7 more)

### Community 90 - "Community 90"
Cohesion: 0.19
Nodes (6): main(), find_fuel_references(), main(), worlds_dir(), test_find_fuel_references(), test_missing_worlds_directory_fails()

### Community 91 - "Community 91"
Cohesion: 0.23
Nodes (13): DISPLAY, XAUTHORITY, environment, environment, environment, environment, environment, services (+5 more)

### Community 92 - "Community 92"
Cohesion: 0.14
Nodes (13): discovery_s, lifecycle, name, optional, state, log_name, service, topics (+5 more)

### Community 93 - "Community 93"
Cohesion: 0.14
Nodes (13): discovery_s, log_name, nodes, name, service, tf_frames, frame, topics (+5 more)

### Community 94 - "Community 94"
Cohesion: 0.22
Nodes (11): assert_px4_params.sh script, compose_exec(), compose_exec_timeout(), compose_ps_status(), compose_setup(), compose_cli.sh script, record_flight.sh script, check_distance_sensor() (+3 more)

### Community 95 - "Community 95"
Cohesion: 0.18
Nodes (6): _flatten(), _json_ready(), _plots_skipped(), write_csv(), write_plots(), write_report()

### Community 97 - "Community 97"
Cohesion: 0.17
Nodes (11): networks, services, x-health, interval, retries, start_period, timeout, x-image (+3 more)

### Community 98 - "Community 98"
Cohesion: 0.17
Nodes (12): NAV2, condition, command, depends_on, hostname, image, privileged, profiles (+4 more)

### Community 99 - "Community 99"
Cohesion: 0.17
Nodes (12): with, compression-level, driver, fetch-depth, if-no-files-found, name, password, path (+4 more)

### Community 100 - "Community 100"
Cohesion: 0.17
Nodes (12): deadband_velocity, feedback, max_accel, max_decel, max_velocity, min_velocity, odom_duration, scale_velocities (+4 more)

### Community 101 - "Community 101"
Cohesion: 0.55
Nodes (11): interval, retries, start_period, test, timeout, healthcheck, healthcheck, healthcheck (+3 more)

### Community 102 - "Community 102"
Cohesion: 0.29
Nodes (11): driver, options, logging, logging, max-file, max-size, logging, logging (+3 more)

### Community 103 - "Community 103"
Cohesion: 0.25
Nodes (11): enabled, observation_sources, plugin, pointcloud, clearing, data_type, marking, max_obstacle_height (+3 more)

### Community 104 - "Community 104"
Cohesion: 0.18
Nodes (11): deadband_velocity, feedback, max_accel, max_decel, max_velocity, min_velocity, odom_duration, scale_velocities (+3 more)

### Community 105 - "Community 105"
Cohesion: 0.25
Nodes (11): enabled, observation_sources, plugin, pointcloud, clearing, data_type, marking, max_obstacle_height (+3 more)

### Community 106 - "Community 106"
Cohesion: 0.18
Nodes (11): deadband_velocity, feedback, max_accel, max_decel, max_velocity, min_velocity, odom_duration, scale_velocities (+3 more)

### Community 107 - "Community 107"
Cohesion: 0.18
Nodes (11): deadband_velocity, feedback, max_accel, max_decel, max_velocity, min_velocity, odom_duration, scale_velocities (+3 more)

### Community 108 - "Community 108"
Cohesion: 0.20
Nodes (10): PX4, Rtabmap, StatePublisher, depends_on, condition, depends_on, condition, depends_on (+2 more)

### Community 109 - "Community 109"
Cohesion: 0.38
Nodes (10): steps, steps, steps, steps, id, if, name, run (+2 more)

### Community 110 - "Community 110"
Cohesion: 0.20
Nodes (10): voxel_layer, enabled, mark_threshold, max_obstacle_height, observation_sources, origin_z, plugin, publish_voxel_map (+2 more)

### Community 111 - "Community 111"
Cohesion: 0.20
Nodes (10): controller_server, ros__parameters, controller_frequency, controller_plugins, failure_tolerance, goal_checker_plugins, min_theta_velocity_threshold, min_x_velocity_threshold (+2 more)

### Community 112 - "Community 112"
Cohesion: 0.20
Nodes (10): voxel_layer, enabled, mark_threshold, max_obstacle_height, observation_sources, origin_z, plugin, publish_voxel_map (+2 more)

### Community 114 - "Community 114"
Cohesion: 0.31
Nodes (5): parse_camera_type(), test_environment_camera_type(), test_flag_overrides_the_environment(), test_missing_or_unknown_camera_type_is_rejected(), test_positional_camera_type()

### Community 115 - "Community 115"
Cohesion: 0.24
Nodes (9): DISPLAY, GALLIUM_DRIVER, HEADLESS, LIBGL_ALWAYS_SOFTWARE, note_unhealthy(), QT_QPA_PLATFORM, RTABMAPVIZ, compose_stack.sh script (+1 more)

### Community 116 - "Community 116"
Cohesion: 0.22
Nodes (9): command, hostname, image, networks, profiles, restart, tty, volumes (+1 more)

### Community 117 - "Community 117"
Cohesion: 0.22
Nodes (9): command, hostname, image, ports, privileged, restart, tty, volumes (+1 more)

### Community 118 - "Community 118"
Cohesion: 0.22
Nodes (9): command, hostname, image, privileged, profiles, restart, tty, volumes (+1 more)

### Community 119 - "Community 119"
Cohesion: 0.22
Nodes (9): StatePublisher, command, hostname, image, networks, privileged, restart, tty (+1 more)

### Community 120 - "Community 120"
Cohesion: 0.22
Nodes (8): DISPLAY, GALLIUM_DRIVER, GZ_HEADLESS_RENDERING, GZ_USE_XVFB, LIBGL_ALWAYS_SOFTWARE, environment, services, PX4

### Community 121 - "Community 121"
Cohesion: 0.22
Nodes (9): ros__parameters, controller_frequency, controller_plugins, failure_tolerance, goal_checker_plugins, min_theta_velocity_threshold, min_x_velocity_threshold, min_y_velocity_threshold (+1 more)

### Community 122 - "Community 122"
Cohesion: 0.22
Nodes (9): allow_unknown, plugin, tolerance, use_astar, planner_server, ros__parameters, expected_planner_frequency, GridBased (+1 more)

### Community 123 - "Community 123"
Cohesion: 0.22
Nodes (9): loop_rate, stop_on_failure, wait_at_waypoint, waypoint_task_executor_plugin, enabled, plugin, waypoint_pause_duration, waypoint_follower (+1 more)

### Community 124 - "Community 124"
Cohesion: 0.22
Nodes (9): allow_unknown, plugin, tolerance, use_astar, planner_server, ros__parameters, expected_planner_frequency, GridBased (+1 more)

### Community 125 - "Community 125"
Cohesion: 0.22
Nodes (9): loop_rate, stop_on_failure, wait_at_waypoint, waypoint_task_executor_plugin, enabled, plugin, waypoint_pause_duration, waypoint_follower (+1 more)

### Community 126 - "Community 126"
Cohesion: 0.22
Nodes (9): ros__parameters, controller_frequency, controller_plugins, failure_tolerance, goal_checker_plugins, min_theta_velocity_threshold, min_x_velocity_threshold, min_y_velocity_threshold (+1 more)

### Community 127 - "Community 127"
Cohesion: 0.22
Nodes (9): allow_unknown, plugin, tolerance, use_astar, planner_server, ros__parameters, expected_planner_frequency, GridBased (+1 more)

### Community 128 - "Community 128"
Cohesion: 0.22
Nodes (9): loop_rate, stop_on_failure, wait_at_waypoint, waypoint_task_executor_plugin, enabled, plugin, waypoint_pause_duration, waypoint_follower (+1 more)

### Community 129 - "Community 129"
Cohesion: 0.22
Nodes (9): allow_unknown, plugin, tolerance, use_astar, planner_server, ros__parameters, expected_planner_frequency, GridBased (+1 more)

### Community 130 - "Community 130"
Cohesion: 0.22
Nodes (9): loop_rate, stop_on_failure, wait_at_waypoint, waypoint_task_executor_plugin, enabled, plugin, waypoint_pause_duration, waypoint_follower (+1 more)

### Community 131 - "Community 131"
Cohesion: 0.25
Nodes (8): on, pull_request, push, workflow_dispatch, paths, branches, paths, tags

### Community 132 - "Community 132"
Cohesion: 0.29
Nodes (5): append_jsonl(), default_log_dir(), load_yaml(), main(), parse_args()

### Community 134 - "Community 134"
Cohesion: 0.25
Nodes (8): simple_smoother, smoother_plugins, do_refinement, max_its, plugin, tolerance, smoother_server, ros__parameters

### Community 135 - "Community 135"
Cohesion: 0.25
Nodes (8): simple_smoother, smoother_plugins, do_refinement, max_its, plugin, tolerance, smoother_server, ros__parameters

### Community 136 - "Community 136"
Cohesion: 0.25
Nodes (8): simple_smoother, smoother_plugins, do_refinement, max_its, plugin, tolerance, smoother_server, ros__parameters

### Community 137 - "Community 137"
Cohesion: 0.38
Nodes (4): DockerBuild.sh script, load_repo_env(), load_env.sh script, up.sh script

### Community 138 - "Community 138"
Cohesion: 0.29
Nodes (7): needs, outputs, runs-on, timeout-minutes, build, image_mode, image_ref

### Community 139 - "Community 139"
Cohesion: 0.29
Nodes (5): COMPOSE_FILES, COMPOSE_PROFILES, DISPLAY, HEADLESS, RTABMAPVIZ

### Community 140 - "Community 140"
Cohesion: 0.33
Nodes (5): log_name, processes, name, pattern, service

### Community 141 - "Community 141"
Cohesion: 0.33
Nodes (5): log_name, processes, name, pattern, service

### Community 142 - "Community 142"
Cohesion: 0.60
Nodes (5): install_px4_control_params.bash script, install_px4_control_params(), _px4_control_install_airframe_posts(), _px4_control_number(), _px4_control_write_failsafe_post()

### Community 144 - "Community 144"
Cohesion: 0.47
Nodes (6): fail_timeout(), fly_until_done(), note_summary(), run_attempt(), snapshot_attempt(), within_total()

### Community 145 - "Community 145"
Cohesion: 0.40
Nodes (3): effective_min_rate(), judge_rate(), snapshot_message()

### Community 146 - "Community 146"
Cohesion: 0.40
Nodes (5): simple_smoother, do_refinement, max_its, plugin, tolerance

### Community 147 - "Community 147"
Cohesion: 0.40
Nodes (5): plugin, stateful, xy_goal_tolerance, yaw_goal_tolerance, general_goal_checker

### Community 148 - "Community 148"
Cohesion: 0.40
Nodes (5): plugin, stateful, xy_goal_tolerance, yaw_goal_tolerance, general_goal_checker

### Community 149 - "Community 149"
Cohesion: 0.40
Nodes (5): plugin, stateful, xy_goal_tolerance, yaw_goal_tolerance, general_goal_checker

### Community 150 - "Community 150"
Cohesion: 0.70
Nodes (4): tmux-session.sh script, Tmux_attach_session(), Tmux_create_session_run_CMDs(), Tmux_kill_session()

### Community 151 - "Community 151"
Cohesion: 0.60
Nodes (3): arg_default(), check_arg(), check_versions.sh script

### Community 152 - "Community 152"
Cohesion: 0.50
Nodes (3): failure-threshold, ignored, trustedRegistries

### Community 153 - "Community 153"
Cohesion: 0.50
Nodes (4): movement_time_allowance, plugin, required_movement_radius, progress_checker

### Community 154 - "Community 154"
Cohesion: 0.50
Nodes (4): movement_time_allowance, plugin, required_movement_radius, progress_checker

### Community 155 - "Community 155"
Cohesion: 0.50
Nodes (4): enabled, observation_sources, plugin, obstacle_layer

### Community 156 - "Community 156"
Cohesion: 0.50
Nodes (4): movement_time_allowance, plugin, required_movement_radius, progress_checker

### Community 157 - "Community 157"
Cohesion: 0.50
Nodes (3): ESTIMATION_MODE, PX4_MAX_YAW_RATE_DEG_S, gz_start_px4_control.sh script

## Knowledge Gaps
- **50 isolated node(s):** `DISPLAY`, `GALLIUM_DRIVER`, `HEADLESS`, `LIBGL_ALWAYS_SOFTWARE`, `QT_QPA_PLATFORM` (+45 more)
  These have ≤1 connection - possible missing edges. (Counts symbols only; 1599 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **46 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Px4ControlNode` connect `Community 1` to `Community 2`, `Community 36`, `Community 4`, `Community 40`, `Community 41`, `Community 15`, `Community 80`, `Community 79`, `Community 143`, `Community 21`, `Community 58`, `Community 28`?**
  _High betweenness centrality (0.032) - this node is a cross-community bridge._
- **Are the 10 inferred relationships involving `Px4ControlNode` (e.g. with `ArmDecision` and `HeightGuard`) actually correct?**
  _`Px4ControlNode` has 10 INFERRED edges - model-reasoned connections that need verification._
- **What connects `DISPLAY`, `GALLIUM_DRIVER`, `HEADLESS` to the rest of the system?**
  _50 weakly-connected nodes found - possible missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.060718252499074414 - nodes in this community are weakly interconnected._
- **Why does `test_headless_hover()` connect `Community 8` to `Community 2`, `Community 71`?**
  _High betweenness centrality (0.022) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `MotionExecutive` (e.g. with `BrakeThenHold` and `Px4ControlNode`) actually correct?**
  _`MotionExecutive` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Should `Community 1` be split into smaller, more focused modules?**
  _Cohesion score 0.06605222734254992 - nodes in this community are weakly interconnected._