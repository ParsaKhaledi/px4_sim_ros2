# Graph Report - px4_sim_ros2  (2026-11-13)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 3390 nodes · 6603 edges · 203 communities (161 shown, 42 thin omitted)
- Extraction: 96% EXTRACTED · 4% INFERRED · 0% AMBIGUOUS · INFERRED: 247 edges (avg confidence: 0.84)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `9ced7684`
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
- Community 93
- Community 94
- Community 95
- Community 96
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
- Community 143
- Community 144
- Community 145
- Community 146
- Community 147
- Community 148
- Community 149
- Community 151
- Community 152
- Community 153
- Community 154
- Community 155
- Community 156
- Community 157
- Community 158
- Community 159
- Community 160
- Community 161
- Community 162
- Community 163
- Community 164
- Community 166
- Community 167
- Community 168
- Community 169
- Community 170
- Community 171
- Community 172
- Community 173
- Community 174
- Community 175
- Community 176
- Community 177
- Community 178
- Community 179
- Community 180
- Community 181
- Community 183
- Community 187
- Community 188
- Community 189
- Community 190
- Community 191
- Community 192
- Community 193
- Community 194
- Community 195
- Community 199
- Community 200
- Community 201

## God Nodes (most connected - your core abstractions)
1. `Px4ControlNode` - 60 edges
2. `MotionExecutive` - 56 edges
3. `FlightWatch` - 51 edges
4. `Snapshot` - 40 edges
5. `FollowPath` - 40 edges
6. `FollowPath` - 39 edges
7. `FollowPath` - 39 edges
8. `FollowPath` - 39 edges
9. `ros__parameters` - 39 edges
10. `ros__parameters` - 39 edges

## Surprising Connections (you probably didn't know these)
- `test_headless_hover()` --references--> `flight`  [EXTRACTED]
  tests/test_hover_flight.py → .github/workflows/ci.yml
- `DockerBuild.sh script` --calls--> `load_repo_env()`  [EXTRACTED]
  DockerBuild.sh → scripts/load_env.sh
- `Px4ControlNode` --uses--> `OdomSample`  [INFERRED]
  ros2_ws/src/px4_control/px4_control/node.py → ros2_ws/src/px4_control/px4_control/vision_bridge.py
- `Px4ControlNode` --uses--> `VisionOdometryBridge`  [INFERRED]
  ros2_ws/src/px4_control/px4_control/node.py → ros2_ws/src/px4_control/px4_control/vision_bridge.py
- `FlightWatch` --uses--> `CameraTracker`  [INFERRED]
  tests/e2e/px4_offboard.py → tests/e2e/camera_check.py

## Import Cycles
- None detected.

## Communities (203 total, 42 thin omitted)

### Community 0 - "Community 0"
Cohesion: 0.05
Nodes (48): height_abort_log(), HeightGuard, wrap_pi(), grade_hover(), HoverGrade, GoalStatus, Limits, MotionExecutive (+40 more)

### Community 1 - "Community 1"
Cohesion: 0.06
Nodes (73): resources, capabilities, count, driver, CAM_COLOR_HEIGHT, CAM_COLOR_WIDTH, CAM_PITCH_DEG, CAM_RATE_HZ (+65 more)

### Community 2 - "Community 2"
Cohesion: 0.07
Nodes (35): attitude_from_enu_quat(), body_horizontal_to_ned(), body_offset_to_enu(), command_age(), covariance_block(), diagonal_variances(), enu_flu_to_ned_frd_rotation(), enu_to_ned() (+27 more)

### Community 3 - "Community 3"
Cohesion: 0.06
Nodes (5): ArmDecision, Px4ControlNode, feedback(), feedback(), feedback()

### Community 4 - "Community 4"
Cohesion: 0.08
Nodes (57): resources, CAM_COLOR_HEIGHT, CAM_COLOR_WIDTH, CAM_PITCH_DEG, CAM_RATE_HZ, CAM_STEREO_HEIGHT, CAM_STEREO_WIDTH, CAM_X (+49 more)

### Community 5 - "Community 5"
Cohesion: 0.10
Nodes (37): _along(), _check(), grade_hover(), _grade_hover_checks(), _grade_hover_clock(), _grade_leg(), grade_mission(), _grade_yaw() (+29 more)

### Community 6 - "Community 6"
Cohesion: 0.11
Nodes (24): _line(), test_align_se3_recovers_rigid_transform_without_scale(), test_constant_offset_is_removed_by_takeoff_frame_and_by_alignment(), test_gps_noise_on_a_biased_cloud(), test_lateral_drift_rpe_and_final_error(), test_yaw_error_detects_a_growing_heading(), wrap_pi(), absolute_trajectory_error() (+16 more)

### Community 7 - "Community 7"
Cohesion: 0.09
Nodes (35): main(), mismatches(), parse_param_value(), apply(), clear_parameter_store(), default_params_dir(), ensure_post_before_ekf2(), env_overrides() (+27 more)

### Community 8 - "Community 8"
Cohesion: 0.07
Nodes (9): parse_camera_type(), Drone, main(), run(), main(), test_environment_camera_type(), test_flag_overrides_the_environment(), test_missing_or_unknown_camera_type_is_rejected() (+1 more)

### Community 9 - "Community 9"
Cohesion: 0.11
Nodes (20): _finite_vector(), _invalid_diagonal(), _is_null_pose(), lost_covariance(), OdomSample, VisionOdometryBridge, VisualOdom, _Msg (+12 more)

### Community 10 - "Community 10"
Cohesion: 0.10
Nodes (27): check_max(), check_min(), distance_sensor_result(), distance_sensor_topic(), header_stamp_s(), lidar_down_enabled(), line(), match_versioned_topic() (+19 more)

### Community 11 - "Community 11"
Cohesion: 0.25
Nodes (42): CAM_COLOR_HEIGHT, CAM_COLOR_WIDTH, CAM_PITCH_DEG, CAM_RATE_HZ, CAM_STEREO_HEIGHT, CAM_STEREO_WIDTH, CAM_X, CAM_Y (+34 more)

### Community 12 - "Community 12"
Cohesion: 0.09
Nodes (20): candidate_base_models(), candidate_server_configs(), candidate_worlds(), ensure_model_systems(), main(), patch_model(), plugin_block(), removal_line() (+12 more)

### Community 13 - "Community 13"
Cohesion: 0.11
Nodes (26): box_inertia(), hfov_from_intrinsics(), Mount, mount_from_env(), pose_text(), sensor_layouts(), _axis_noise(), _block_span() (+18 more)

### Community 14 - "Community 14"
Cohesion: 0.05
Nodes (40): acc_lim_theta, acc_lim_x, acc_lim_y, angular_granularity, BaseObstacle.scale, BaseObstacle.sum_scores, critics, debug_trajectory_details (+32 more)

### Community 15 - "Community 15"
Cohesion: 0.09
Nodes (20): apply_origin_to_worlds(), ecef_to_geodetic(), enu_to_geodetic(), geodetic_to_ecef(), main(), origin_from_env(), parse_spawn_xyz(), px4_world_dirs() (+12 more)

### Community 16 - "Community 16"
Cohesion: 0.09
Nodes (24): build_parser(), _env_float(), fail_on_loss_enabled(), inlier_gate_value(), interval_rate(), json_number(), load_env_file(), load_repo_env() (+16 more)

### Community 17 - "Community 17"
Cohesion: 0.05
Nodes (39): acc_lim_theta, acc_lim_x, acc_lim_y, angular_granularity, BaseObstacle.scale, critics, debug_trajectory_details, decel_lim_theta (+31 more)

### Community 18 - "Community 18"
Cohesion: 0.05
Nodes (39): acc_lim_theta, acc_lim_x, acc_lim_y, angular_granularity, BaseObstacle.scale, critics, debug_trajectory_details, decel_lim_theta (+31 more)

### Community 19 - "Community 19"
Cohesion: 0.05
Nodes (39): acc_lim_theta, acc_lim_x, acc_lim_y, angular_granularity, BaseObstacle.scale, critics, debug_trajectory_details, decel_lim_theta (+31 more)

### Community 20 - "Community 20"
Cohesion: 0.05
Nodes (37): x-ros-env, CAM_COLOR_HEIGHT, CAM_COLOR_WIDTH, CAM_PITCH_DEG, CAM_RATE_HZ, CAM_STEREO_HEIGHT, CAM_STEREO_WIDTH, CAM_X (+29 more)

### Community 21 - "Community 21"
Cohesion: 0.05
Nodes (37): ros__parameters, alpha1, alpha2, alpha3, alpha4, alpha5, base_frame_id, beam_skip_distance (+29 more)

### Community 22 - "Community 22"
Cohesion: 0.05
Nodes (37): ros__parameters, alpha1, alpha2, alpha3, alpha4, alpha5, base_frame_id, beam_skip_distance (+29 more)

### Community 23 - "Community 23"
Cohesion: 0.05
Nodes (37): ros__parameters, alpha1, alpha2, alpha3, alpha4, alpha5, base_frame_id, beam_skip_distance (+29 more)

### Community 24 - "Community 24"
Cohesion: 0.11
Nodes (19): download_fuel_file(), _download_named(), _ensure_model_config(), _follow_local_sidecars(), _follow_mesh_sidecars(), fuel_model_version(), _get(), main() (+11 more)

### Community 25 - "Community 25"
Cohesion: 0.11
Nodes (18): flu_enu_quaternion(), flu_enu_quaternion_xyzw(), frd_to_flu(), gravity_quaternion_xyzw(), quat_from_rpy(), quat_mul(), quat_normalize(), quat_to_rot() (+10 more)

### Community 26 - "Community 26"
Cohesion: 0.12
Nodes (16): heartbeat(), _ident(), param_request_read(), param_set(), ParamValue, parse_frames(), _parse_param_value(), read_params() (+8 more)

### Community 27 - "Community 27"
Cohesion: 0.09
Nodes (20): accel_stddev_m_s2(), _add(), _allowed_stereo_text(), _env_int(), gpu_visible(), gyro_stddev_rad_s(), matmul(), matvec() (+12 more)

### Community 28 - "Community 28"
Cohesion: 0.06
Nodes (33): px4_control, ros__parameters, a_brake, arm_timeout_s, cmd_vel_timeout_s, cmd_vel_topic, e2e_height_tolerance_m, estimation_mode (+25 more)

### Community 29 - "Community 29"
Cohesion: 0.12
Nodes (16): test_geodetic_equator_offsets(), test_level_body_frd_aligned_with_ned_points_north_in_enu(), test_ned_position_to_enu(), test_yaw_only_ned_to_enu(), ecef_to_enu(), geodetic_to_ecef(), geodetic_to_enu(), ned_frd_to_enu_flu() (+8 more)

### Community 30 - "Community 30"
Cohesion: 0.08
Nodes (15): ack_failure_text(), combine_failure(), failsafe_reasons(), _magnetometer_arming_text(), prearm_block_reason(), sim_preflight_decision(), _fill3(), estimation_mode_from_env() (+7 more)

### Community 31 - "Community 31"
Cohesion: 0.12
Nodes (14): corrected_projection(), _env_float(), no_gpu_warning(), profile_from_env(), rectified_k(), rectified_p(), stereo_intrinsics(), stereo_tx() (+6 more)

### Community 33 - "Community 33"
Cohesion: 0.12
Nodes (15): airframe_name(), _ensure_covariance(), ground_truth_model_names(), main(), patch_file(), patch_px4_spawned_models(), px4_model_sdf(), px4_models_roots() (+7 more)

### Community 34 - "Community 34"
Cohesion: 0.07
Nodes (30): cleanup(), E2E_ATTEMPT_TIMEOUT, E2E_CAMERA_MIN_VARIANCE, E2E_CHECK_CAMERAS, E2E_CRASH_DIVERGENCE_M, E2E_CRASH_IMPACT_SPEED_MPS, E2E_CRASH_MIN_HEIGHT_M, E2E_CRASH_TILT_DEG (+22 more)

### Community 35 - "Community 35"
Cohesion: 0.12
Nodes (10): check_rate(), gated_rate_hz(), RateTracker, _span_hz(), stale_limit_s(), topic_rate_line(), _constant_imu(), test_rate_tracker() (+2 more)

### Community 36 - "Community 36"
Cohesion: 0.10
Nodes (27): bt_navigator, ros__parameters, ros__parameters, cost_scaling_factor, inflation_radius, plugin, ros__parameters, always_send_full_costmap (+19 more)

### Community 37 - "Community 37"
Cohesion: 0.15
Nodes (19): test_gps_field_layouts(), _odom(), read_bag(), gps_fields_to_lla(), quat_xyzw_to_wxyz(), _odom_sample(), record_live(), _capture_spawn() (+11 more)

### Community 38 - "Community 38"
Cohesion: 0.12
Nodes (3): setpoint_world(), FlightWatch, at_height()

### Community 39 - "Community 39"
Cohesion: 0.11
Nodes (26): ros__parameters, ros__parameters, cost_scaling_factor, inflation_radius, plugin, ros__parameters, always_send_full_costmap, bt_loop_duration (+18 more)

### Community 40 - "Community 40"
Cohesion: 0.11
Nodes (26): ros__parameters, ros__parameters, cost_scaling_factor, inflation_radius, plugin, ros__parameters, always_send_full_costmap, bt_loop_duration (+18 more)

### Community 41 - "Community 41"
Cohesion: 0.11
Nodes (26): ros__parameters, ros__parameters, cost_scaling_factor, inflation_radius, plugin, ros__parameters, always_send_full_costmap, bt_loop_duration (+18 more)

### Community 42 - "Community 42"
Cohesion: 0.12
Nodes (15): ground_truth_leak_from_info(), gz_topic_publishers(), px4_vision_covariance_topic(), px4_vision_model_name(), relative_position_error(), summarize(), camera_info_topic(), check_ground_truth_vision_leak() (+7 more)

### Community 43 - "Community 43"
Cohesion: 0.11
Nodes (11): _px4_qos(), load_px4_topics(), message_version(), _parse_topic_file(), select_live_topic(), subscription_names(), topic_candidates(), topics_config_path() (+3 more)

### Community 44 - "Community 44"
Cohesion: 0.09
Nodes (24): amcl, behavior_server, bt_navigator, bt_navigator_navigate_through_poses_rclcpp_node, ros__parameters, bt_navigator_navigate_to_pose_rclcpp_node, ros__parameters, global_costmap (+16 more)

### Community 45 - "Community 45"
Cohesion: 0.09
Nodes (24): amcl, behavior_server, bt_navigator, bt_navigator_navigate_through_poses_rclcpp_node, ros__parameters, bt_navigator_navigate_to_pose_rclcpp_node, ros__parameters, global_costmap (+16 more)

### Community 46 - "Community 46"
Cohesion: 0.09
Nodes (24): behavior_server, bt_navigator, bt_navigator_navigate_through_poses_rclcpp_node, ros__parameters, bt_navigator_navigate_to_pose_rclcpp_node, ros__parameters, controller_server, global_costmap (+16 more)

### Community 47 - "Community 47"
Cohesion: 0.15
Nodes (15): should_retry(), build_report(), crash_reset_count(), load_attempts(), main(), needs_render_fallback(), write_report(), write_stub() (+7 more)

### Community 48 - "Community 48"
Cohesion: 0.13
Nodes (15): yaw_from_quat(), _block_value(), match_score(), ned_from_enu(), _one_pose(), parse_pose_v(), parse_rtf(), _pose_blocks() (+7 more)

### Community 49 - "Community 49"
Cohesion: 0.14
Nodes (12): default_min_rtf(), _env_float(), expected_imu_hz(), expected_sensor_hz(), _load_geometry(), minimum_rate_hz(), _oak_model_imu_hz(), _profile_imu_hz() (+4 more)

### Community 50 - "Community 50"
Cohesion: 0.17
Nodes (7): child_text(), findall(), load_module(), local(), pose_values(), RenderTest, sensor_pose()

### Community 51 - "Community 51"
Cohesion: 0.14
Nodes (13): candidate_models(), covariance_topic_problems(), patch_text(), _plugin_blocks(), repo_x500_depth_sdf(), update_repo_model(), main(), test_candidate_model_paths_are_unique() (+5 more)

### Community 52 - "Community 52"
Cohesion: 0.12
Nodes (12): bridge_gazebo_imu(), configured_imu_publishers(), diagonal_covariance(), imu_source(), imu_stamp_ns(), measure_px4_offset_ns(), normalize_imu_source(), normalize_stamp_mode() (+4 more)

### Community 53 - "Community 53"
Cohesion: 0.17
Nodes (4): FlightCrash, down(), held(), inside()

### Community 54 - "Community 54"
Cohesion: 0.09
Nodes (22): discovery_s, gz_rtf, enabled, log_name, processes, name, pattern, service (+14 more)

### Community 55 - "Community 55"
Cohesion: 0.15
Nodes (12): format_rate(), _has_lidar(), lidar_enabled(), lidar_fragment(), lidar_rate_hz(), patch_file(), patch_text(), _remove_lidar_elements() (+4 more)

### Community 56 - "Community 56"
Cohesion: 0.12
Nodes (4): parse_args(), str2bool(), FakeVisionSource, main()

### Community 57 - "Community 57"
Cohesion: 0.11
Nodes (22): enabled, observation_sources, plugin, pointcloud, clearing, data_type, marking, max_obstacle_height (+14 more)

### Community 58 - "Community 58"
Cohesion: 0.11
Nodes (21): amcl, behavior_server, bt_navigator_navigate_through_poses_rclcpp_node, ros__parameters, bt_navigator_navigate_to_pose_rclcpp_node, ros__parameters, controller_server, global_costmap (+13 more)

### Community 59 - "Community 59"
Cohesion: 0.12
Nodes (12): consume_stats_line(), find_stats_topic(), iter_real_time_factors(), main(), _ensure_stream(), __init__(), _read(), parse_stats_message() (+4 more)

### Community 60 - "Community 60"
Cohesion: 0.18
Nodes (11): lidar_height_log(), LidarHeightGuard, _feed(), test_a_sample_back_inside_the_tolerance_resets_the_timer(), test_four_tenths_disagreement_for_six_tenths_aborts(), test_gap_must_last_more_than_the_duration(), test_no_distance_sensor_logs_once(), test_on_the_ground_below_min_range_does_not_abort() (+3 more)

### Community 61 - "Community 61"
Cohesion: 0.22
Nodes (6): HealthRunner, import_message(), process_alive(), qos_profile(), topic_matches_env(), utc_now()

### Community 62 - "Community 62"
Cohesion: 0.10
Nodes (20): plugin, plugin, ros__parameters, plugin, assisted_teleop, backup, behavior_plugins, costmap_topic (+12 more)

### Community 63 - "Community 63"
Cohesion: 0.10
Nodes (20): plugin, plugin, ros__parameters, plugin, assisted_teleop, backup, behavior_plugins, costmap_topic (+12 more)

### Community 64 - "Community 64"
Cohesion: 0.10
Nodes (20): plugin, plugin, ros__parameters, plugin, assisted_teleop, backup, behavior_plugins, costmap_topic (+12 more)

### Community 65 - "Community 65"
Cohesion: 0.10
Nodes (20): plugin, plugin, ros__parameters, plugin, assisted_teleop, backup, behavior_plugins, costmap_topic (+12 more)

### Community 66 - "Community 66"
Cohesion: 0.11
Nodes (18): runs-on, timeout-minutes, concurrency, cancel-in-progress, group, needs, runs-on, timeout-minutes (+10 more)

### Community 67 - "Community 67"
Cohesion: 0.20
Nodes (11): FlightSetupError, run_px4_mission(), _call(), driver_name(), _dump(), import_drone(), main(), _result_path() (+3 more)

### Community 68 - "Community 68"
Cohesion: 0.20
Nodes (10): camera_overrides(), effective_parameters(), ini_path(), launch_strings(), load_profile_ini(), main(), parse_rtabmap_ini(), _rate_text() (+2 more)

### Community 69 - "Community 69"
Cohesion: 0.18
Nodes (8): test_offset_is_the_lowest_gap_after_fifty_samples(), test_short_recording_leaves_the_offset_empty(), test_written_file_has_only_the_agreed_fields(), test_yaw_from_the_spawn_quaternion(), _xyzw(), build_spawn_document(), write_spawn_json(), yaw_from_xyzw()

### Community 70 - "Community 70"
Cohesion: 0.18
Nodes (11): CameraTracker, _channel_samples(), is_blank(), pixel_stats(), rtf_summary(), test_a_flat_frame_is_blank(), test_a_varied_frame_is_not_blank(), test_depth_samples_are_little_endian() (+3 more)

### Community 71 - "Community 71"
Cohesion: 0.16
Nodes (9): read_gz_rtf(), _run(), test_script_dir_parent_wins_when_the_tree_is_there(), test_sim_gz_dir_override_and_missing_tree(), _install(), test_installer_keeps_full_sim_params_post(), test_installer_writes_airframe_post_for_nav_dll(), test_param_env_is_idempotent_and_selects_mode() (+1 more)

### Community 72 - "Community 72"
Cohesion: 0.14
Nodes (18): pointcloud, clearing, data_type, marking, max_obstacle_height, min_obstacle_height, topic, voxel_layer (+10 more)

### Community 73 - "Community 73"
Cohesion: 0.18
Nodes (10): test_nav2_frames_match_rtabmap_base_link(), _command_text(), _config(), _config_env(), test_camera_env_follows_the_shell(), test_ci_override_is_headless(), test_cpu_and_gpu_pass_the_same_camera_env(), test_every_compose_file_parses() (+2 more)

### Community 74 - "Community 74"
Cohesion: 0.17
Nodes (7): test_resample_pairs_different_rates(), _add_topic_args(), _build_parser(), main(), _print_summary(), read_tum(), write_tum()

### Community 75 - "Community 75"
Cohesion: 0.16
Nodes (8): CrashThresholds, load_crash_thresholds(), error_stats(), gazebo_env(), read_gz_pose(), read_gz_rtf(), _threshold_dump(), wrap_pi()

### Community 76 - "Community 76"
Cohesion: 0.12
Nodes (17): CameraType, COMPOSE_FILES, COMPOSE_PROFILES, COMPOSE_PROJECT_NAME, COMPOSE_SERVICES, DISPLAY, GH_CACHE_TOKEN, GZ_HEADLESS_RENDERING (+9 more)

### Community 77 - "Community 77"
Cohesion: 0.12
Nodes (15): discovery_s, log_name, nodes, name, optional, service, topics, group (+7 more)

### Community 78 - "Community 78"
Cohesion: 0.21
Nodes (11): test_absent_optional_topic_skips(), test_auto_detect_after_measure_prefers_live_unversioned_odometry(), test_auto_detect_falls_back_to_existing_topic(), test_auto_detect_prefers_publisher(), test_auto_detect_prefers_version_when_neither_has_a_publisher(), test_camera_env_filter(), test_check_files_cover_services(), test_ground_truth_floor_tracks_the_sim_clock() (+3 more)

### Community 79 - "Community 79"
Cohesion: 0.15
Nodes (12): gz_resolve_dir(), gz_resolve_dir.sh script, GZ_IP, GZ_SIM_RESOURCE_PATH, PATH, PX4_GZ_MODEL_POSE, PX4_GZ_WORLD, PX4_SIM_MODEL (+4 more)

### Community 80 - "Community 80"
Cohesion: 0.12
Nodes (15): min-spaces-from-content, extends, ignore, indent-sequences, spaces, max, rules, braces (+7 more)

### Community 81 - "Community 81"
Cohesion: 0.17
Nodes (5): ModeCommand, OffboardStreamGate, parse_mode(), test_offboard_burst_is_not_a_warm_stream(), test_parse_mode_offboard()

### Community 83 - "Community 83"
Cohesion: 0.17
Nodes (6): _env_float(), jumped_to_identity(), __init__(), _norm(), PoseOrigins, test_pose_error_uses_ground_truth_when_rtabmap_starts()

### Community 84 - "Community 84"
Cohesion: 0.23
Nodes (13): DISPLAY, XAUTHORITY, environment, environment, environment, environment, environment, services (+5 more)

### Community 85 - "Community 85"
Cohesion: 0.14
Nodes (13): discovery_s, lifecycle, name, optional, state, log_name, service, topics (+5 more)

### Community 86 - "Community 86"
Cohesion: 0.14
Nodes (13): discovery_s, log_name, nodes, name, service, tf_frames, frame, topics (+5 more)

### Community 87 - "Community 87"
Cohesion: 0.22
Nodes (6): parse_spawn_pose(), quat_from_rpy(), main(), __init__(), spawn_ground_pose(), test_spawn_pose_default()

### Community 88 - "Community 88"
Cohesion: 0.22
Nodes (11): assert_px4_params.sh script, compose_exec(), compose_exec_timeout(), compose_ps_status(), compose_setup(), compose_cli.sh script, record_flight.sh script, check_distance_sensor() (+3 more)

### Community 89 - "Community 89"
Cohesion: 0.18
Nodes (6): _flatten(), _json_ready(), _plots_skipped(), write_csv(), write_plots(), write_report()

### Community 90 - "Community 90"
Cohesion: 0.23
Nodes (7): format_summary(), _hz(), jsonl_events(), main(), count(), summarize(), write_jsonl()

### Community 91 - "Community 91"
Cohesion: 0.23
Nodes (7): height_above_floor(), reading_in_range(), tilt_compensated_height(), test_level_range_is_the_height(), test_mount_offset_is_subtracted_after_tilt_compensation(), test_tilt_uses_the_body_down_cosine(), test_upward_beam_is_not_a_height()

### Community 93 - "Community 93"
Cohesion: 0.17
Nodes (11): networks, services, x-health, interval, retries, start_period, timeout, x-image (+3 more)

### Community 94 - "Community 94"
Cohesion: 0.17
Nodes (12): NAV2, condition, command, depends_on, hostname, image, privileged, profiles (+4 more)

### Community 95 - "Community 95"
Cohesion: 0.17
Nodes (12): with, compression-level, driver, fetch-depth, if-no-files-found, name, password, path (+4 more)

### Community 96 - "Community 96"
Cohesion: 0.26
Nodes (3): lost_from_stats(), pick_inliers(), TrackingLog

### Community 97 - "Community 97"
Cohesion: 0.39
Nodes (10): crash_reason(), _obs(), test_healthy_hover_is_not_a_crash(), test_land_phase_may_be_low_and_disarmed(), test_low_altitude_and_impact_while_airborne(), test_setpoint_divergence_and_odom_timeout(), test_thresholds_come_from_env(), test_tilt_on_the_ground_is_ignored() (+2 more)

### Community 98 - "Community 98"
Cohesion: 0.55
Nodes (11): interval, retries, start_period, test, timeout, healthcheck, healthcheck, healthcheck (+3 more)

### Community 99 - "Community 99"
Cohesion: 0.29
Nodes (11): driver, options, logging, logging, max-file, max-size, logging, logging (+3 more)

### Community 100 - "Community 100"
Cohesion: 0.18
Nodes (5): choose_topic(), effective_min_rate(), judge_rate(), snapshot_message(), topic_version()

### Community 101 - "Community 101"
Cohesion: 0.25
Nodes (11): enabled, observation_sources, plugin, pointcloud, clearing, data_type, marking, max_obstacle_height (+3 more)

### Community 102 - "Community 102"
Cohesion: 0.18
Nodes (11): deadband_velocity, feedback, max_accel, max_decel, max_velocity, min_velocity, odom_duration, scale_velocities (+3 more)

### Community 103 - "Community 103"
Cohesion: 0.18
Nodes (11): deadband_velocity, feedback, max_accel, max_decel, max_velocity, min_velocity, odom_duration, scale_velocities (+3 more)

### Community 104 - "Community 104"
Cohesion: 0.25
Nodes (11): enabled, observation_sources, plugin, pointcloud, clearing, data_type, marking, max_obstacle_height (+3 more)

### Community 105 - "Community 105"
Cohesion: 0.18
Nodes (11): deadband_velocity, feedback, max_accel, max_decel, max_velocity, min_velocity, odom_duration, scale_velocities (+3 more)

### Community 106 - "Community 106"
Cohesion: 0.18
Nodes (11): deadband_velocity, feedback, max_accel, max_decel, max_velocity, min_velocity, odom_duration, scale_velocities (+3 more)

### Community 107 - "Community 107"
Cohesion: 0.20
Nodes (10): PX4, Rtabmap, StatePublisher, depends_on, condition, depends_on, condition, depends_on (+2 more)

### Community 108 - "Community 108"
Cohesion: 0.38
Nodes (10): steps, steps, steps, steps, id, if, name, run (+2 more)

### Community 109 - "Community 109"
Cohesion: 0.36
Nodes (7): close_vec(), main(), check_fx(), check_image(), wait_for(), quat_angle(), record()

### Community 110 - "Community 110"
Cohesion: 0.20
Nodes (10): controller_server, ros__parameters, controller_frequency, controller_plugins, failure_tolerance, goal_checker_plugins, min_theta_velocity_threshold, min_x_velocity_threshold (+2 more)

### Community 111 - "Community 111"
Cohesion: 0.20
Nodes (10): controller_server, ros__parameters, controller_frequency, controller_plugins, failure_tolerance, goal_checker_plugins, min_theta_velocity_threshold, min_x_velocity_threshold (+2 more)

### Community 112 - "Community 112"
Cohesion: 0.20
Nodes (10): voxel_layer, enabled, mark_threshold, max_obstacle_height, observation_sources, origin_z, plugin, publish_voxel_map (+2 more)

### Community 113 - "Community 113"
Cohesion: 0.20
Nodes (5): _color_full(), color_intrinsics(), diagonal_fov_deg(), scale_intrinsics(), vfov_from_intrinsics()

### Community 114 - "Community 114"
Cohesion: 0.20
Nodes (10): voxel_layer, enabled, mark_threshold, max_obstacle_height, observation_sources, origin_z, plugin, publish_voxel_map (+2 more)

### Community 115 - "Community 115"
Cohesion: 0.24
Nodes (9): DISPLAY, GALLIUM_DRIVER, HEADLESS, LIBGL_ALWAYS_SOFTWARE, note_unhealthy(), QT_QPA_PLATFORM, RTABMAPVIZ, compose_stack.sh script (+1 more)

### Community 116 - "Community 116"
Cohesion: 0.22
Nodes (9): command, hostname, image, ports, privileged, restart, tty, volumes (+1 more)

### Community 117 - "Community 117"
Cohesion: 0.22
Nodes (9): command, hostname, image, privileged, profiles, restart, tty, volumes (+1 more)

### Community 118 - "Community 118"
Cohesion: 0.22
Nodes (9): StatePublisher, command, hostname, image, networks, privileged, restart, tty (+1 more)

### Community 119 - "Community 119"
Cohesion: 0.22
Nodes (8): DISPLAY, GALLIUM_DRIVER, GZ_HEADLESS_RENDERING, GZ_USE_XVFB, LIBGL_ALWAYS_SOFTWARE, environment, services, PX4

### Community 120 - "Community 120"
Cohesion: 0.22
Nodes (9): allow_unknown, plugin, tolerance, use_astar, planner_server, ros__parameters, expected_planner_frequency, GridBased (+1 more)

### Community 121 - "Community 121"
Cohesion: 0.22
Nodes (9): loop_rate, stop_on_failure, wait_at_waypoint, waypoint_task_executor_plugin, enabled, plugin, waypoint_pause_duration, waypoint_follower (+1 more)

### Community 122 - "Community 122"
Cohesion: 0.22
Nodes (9): allow_unknown, plugin, tolerance, use_astar, planner_server, ros__parameters, expected_planner_frequency, GridBased (+1 more)

### Community 123 - "Community 123"
Cohesion: 0.22
Nodes (9): loop_rate, stop_on_failure, wait_at_waypoint, waypoint_task_executor_plugin, enabled, plugin, waypoint_pause_duration, waypoint_follower (+1 more)

### Community 124 - "Community 124"
Cohesion: 0.22
Nodes (9): ros__parameters, controller_frequency, controller_plugins, failure_tolerance, goal_checker_plugins, min_theta_velocity_threshold, min_x_velocity_threshold, min_y_velocity_threshold (+1 more)

### Community 125 - "Community 125"
Cohesion: 0.22
Nodes (9): ros__parameters, controller_frequency, controller_plugins, failure_tolerance, goal_checker_plugins, min_theta_velocity_threshold, min_x_velocity_threshold, min_y_velocity_threshold (+1 more)

### Community 126 - "Community 126"
Cohesion: 0.22
Nodes (9): allow_unknown, plugin, tolerance, use_astar, planner_server, ros__parameters, expected_planner_frequency, GridBased (+1 more)

### Community 127 - "Community 127"
Cohesion: 0.22
Nodes (9): loop_rate, stop_on_failure, wait_at_waypoint, waypoint_task_executor_plugin, enabled, plugin, waypoint_pause_duration, waypoint_follower (+1 more)

### Community 128 - "Community 128"
Cohesion: 0.22
Nodes (9): allow_unknown, plugin, tolerance, use_astar, planner_server, ros__parameters, expected_planner_frequency, GridBased (+1 more)

### Community 129 - "Community 129"
Cohesion: 0.22
Nodes (9): loop_rate, stop_on_failure, wait_at_waypoint, waypoint_task_executor_plugin, enabled, plugin, waypoint_pause_duration, waypoint_follower (+1 more)

### Community 130 - "Community 130"
Cohesion: 0.33
Nodes (5): _is_camera(), main(), tune_cameras(), replace(), test_tune_changes_rate_and_leaves_size_and_imu()

### Community 131 - "Community 131"
Cohesion: 0.25
Nodes (8): command, hostname, image, profiles, restart, tty, volumes, NAV2

### Community 132 - "Community 132"
Cohesion: 0.25
Nodes (8): networks, networks, px4, aliases, driver, networks, networks, networks

### Community 133 - "Community 133"
Cohesion: 0.25
Nodes (8): command, hostname, image, profiles, restart, tty, volumes, Rtabmap

### Community 134 - "Community 134"
Cohesion: 0.25
Nodes (8): on, pull_request, push, workflow_dispatch, paths, branches, paths, tags

### Community 135 - "Community 135"
Cohesion: 0.29
Nodes (5): append_jsonl(), default_log_dir(), load_yaml(), main(), parse_args()

### Community 136 - "Community 136"
Cohesion: 0.25
Nodes (8): simple_smoother, smoother_plugins, do_refinement, max_its, plugin, tolerance, smoother_server, ros__parameters

### Community 137 - "Community 137"
Cohesion: 0.25
Nodes (8): simple_smoother, smoother_plugins, do_refinement, max_its, plugin, tolerance, smoother_server, ros__parameters

### Community 138 - "Community 138"
Cohesion: 0.25
Nodes (8): simple_smoother, smoother_plugins, do_refinement, max_its, plugin, tolerance, smoother_server, ros__parameters

### Community 139 - "Community 139"
Cohesion: 0.29
Nodes (4): main(), find_fuel_references(), test_find_fuel_references(), test_missing_worlds_directory_fails()

### Community 140 - "Community 140"
Cohesion: 0.38
Nodes (4): DockerBuild.sh script, load_repo_env(), load_env.sh script, up.sh script

### Community 141 - "Community 141"
Cohesion: 0.29
Nodes (7): needs, outputs, runs-on, timeout-minutes, build, image_mode, image_ref

### Community 142 - "Community 142"
Cohesion: 0.38
Nodes (3): ensure_distance_sensor(), main(), test_patch_adds_the_outbound_topic_once()

### Community 143 - "Community 143"
Cohesion: 0.43
Nodes (4): main(), needs_rebuild(), test_dockerfile_and_manifest_rebuild(), test_runtime_files_do_not_rebuild()

### Community 144 - "Community 144"
Cohesion: 0.29
Nodes (5): COMPOSE_FILES, COMPOSE_PROFILES, DISPLAY, HEADLESS, RTABMAPVIZ

### Community 145 - "Community 145"
Cohesion: 0.33
Nodes (5): log_name, processes, name, pattern, service

### Community 146 - "Community 146"
Cohesion: 0.33
Nodes (5): log_name, processes, name, pattern, service

### Community 147 - "Community 147"
Cohesion: 0.60
Nodes (5): install_px4_control_params.bash script, install_px4_control_params(), _px4_control_install_airframe_posts(), _px4_control_number(), _px4_control_write_failsafe_post()

### Community 148 - "Community 148"
Cohesion: 0.33
Nodes (6): map_saver, ros__parameters, free_thresh_default, map_subscribe_transient_local, occupied_thresh_default, save_map_timeout

### Community 149 - "Community 149"
Cohesion: 0.47
Nodes (6): fail_timeout(), fly_until_done(), note_summary(), run_attempt(), snapshot_attempt(), within_total()

### Community 151 - "Community 151"
Cohesion: 0.40
Nodes (5): plugin, stateful, xy_goal_tolerance, yaw_goal_tolerance, general_goal_checker

### Community 152 - "Community 152"
Cohesion: 0.40
Nodes (5): plugin, stateful, xy_goal_tolerance, yaw_goal_tolerance, general_goal_checker

### Community 153 - "Community 153"
Cohesion: 0.40
Nodes (5): plugin, stateful, xy_goal_tolerance, yaw_goal_tolerance, general_goal_checker

### Community 154 - "Community 154"
Cohesion: 0.40
Nodes (5): simple_smoother, do_refinement, max_its, plugin, tolerance

### Community 155 - "Community 155"
Cohesion: 0.40
Nodes (5): plugin, stateful, xy_goal_tolerance, yaw_goal_tolerance, general_goal_checker

### Community 156 - "Community 156"
Cohesion: 0.70
Nodes (4): tmux-session.sh script, Tmux_attach_session(), Tmux_create_session_run_CMDs(), Tmux_kill_session()

### Community 157 - "Community 157"
Cohesion: 0.60
Nodes (3): arg_default(), check_arg(), check_versions.sh script

### Community 158 - "Community 158"
Cohesion: 0.50
Nodes (3): failure-threshold, ignored, trustedRegistries

### Community 159 - "Community 159"
Cohesion: 0.50
Nodes (4): movement_time_allowance, plugin, required_movement_radius, progress_checker

### Community 160 - "Community 160"
Cohesion: 0.50
Nodes (4): movement_time_allowance, plugin, required_movement_radius, progress_checker

### Community 161 - "Community 161"
Cohesion: 0.50
Nodes (4): movement_time_allowance, plugin, required_movement_radius, progress_checker

### Community 162 - "Community 162"
Cohesion: 0.50
Nodes (4): enabled, observation_sources, plugin, obstacle_layer

### Community 163 - "Community 163"
Cohesion: 0.50
Nodes (4): movement_time_allowance, plugin, required_movement_radius, progress_checker

### Community 164 - "Community 164"
Cohesion: 0.50
Nodes (3): ESTIMATION_MODE, PX4_MAX_YAW_RATE_DEG_S, gz_start_px4_control.sh script

## Knowledge Gaps
- **50 isolated node(s):** `DISPLAY`, `GALLIUM_DRIVER`, `HEADLESS`, `LIBGL_ALWAYS_SOFTWARE`, `QT_QPA_PLATFORM` (+45 more)
  These have ≤1 connection - possible missing edges. (Counts symbols only; 1599 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **42 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `flight` connect `Community 66` to `Community 108`, `Community 5`?**
  _High betweenness centrality (0.039) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `Px4ControlNode` (e.g. with `OdomSample` and `VisionOdometryBridge`) actually correct?**
  _`Px4ControlNode` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `DISPLAY`, `GALLIUM_DRIVER`, `HEADLESS` to the rest of the system?**
  _50 weakly-connected nodes found - possible missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.05271439811172305 - nodes in this community are weakly interconnected._
- **Why does `test_headless_hover()` connect `Community 5` to `Community 66`, `Community 2`?**
  _High betweenness centrality (0.039) - this node is a cross-community bridge._
- **Should `Community 1` be split into smaller, more focused modules?**
  _Cohesion score 0.060718252499074414 - nodes in this community are weakly interconnected._
- **Why does `Px4ControlNode` connect `Community 3` to `Community 0`, `Community 2`, `Community 8`, `Community 9`, `Community 43`, `Community 26`, `Community 60`, `Community 30`?**
  _High betweenness centrality (0.018) - this node is a cross-community bridge._