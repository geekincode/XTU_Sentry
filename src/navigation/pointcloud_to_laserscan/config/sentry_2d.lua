-- Cartographer 2D SLAM configuration for FAST-LIO + LaserScan pipeline
-- 
-- This configuration is optimized for:
-- - 2D laser scan input (from pointcloud_to_laserscan)
-- - FAST-LIO point cloud registration
-- - Real-time mapping with loop closure
-- - Sentry robot platform (ground-based)
--
-- Input topics:
--   /scan - 2D laser scan from pointcloud_to_laserscan
--   /Odometry - Optional odometry (from FAST-LIO)
--   /imu/data - Optional IMU data

include "map_builder.lua"
include "trajectory_builder.lua"

-- Main configuration options
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- Frame configuration
  map_frame = "map",
  tracking_frame = "base_link",        -- Base frame for tracking
  published_frame = "base_link",       -- Frame published to TF
  odom_frame = "odom",                 -- Odometry frame
  provide_odom_frame = true,           -- Publish TF for odom -> base_link
  
  -- 2D-specific settings
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  
  -- Sensor configuration
  use_odometry = true,                 -- Use FAST-LIO odometry
  use_nav_sat = false,                 -- No GPS/GNSS
  use_landmarks = false,               -- No landmark detection
  
  -- Laser scan input (2D)
  num_laser_scans = 1,                 -- Single laser scan input
  num_multi_echo_laser_scans = 0,      -- No multi-echo
  num_subdivisions_per_laser_scan = 1, -- No subdivision - use full scan
  
  -- Point cloud input disabled for 2D
  num_point_clouds = 0,
  
  -- Timing and publishing
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  
  -- Sampling ratios (full sampling for real-time)
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- Enable 2D trajectory builder
MAP_BUILDER.use_trajectory_builder_2d = true

-- Number of accumulated range data scans before processing
-- Lower value = more responsive, higher value = smoother results
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- 2D trajectory builder tuning for FAST-LIO input
TRAJECTORY_BUILDER_2D.motion_filter = {
  max_time_seconds = 0.5,
  max_distance_meters = 0.1,           -- Accumulate motion over 10cm
  max_angle_radians = math.rad(1),     -- Or 1 degree rotation
}

-- Correlative scan matcher parameters
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher = {
  linear_search_window = 0.15,
  angular_search_window = math.rad(15),
  translation_delta_cost_weight = 1e-1,
  rotation_delta_cost_weight = 1e-1,
}

-- CeresScan matcher (optimization-based)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher = {
  occupied_space_weight = 1.,
  translation_weight = 10.,
  rotation_weight = 40.,
  ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 20,
    num_threads = 1,
  },
}

-- Submap configuration
TRAJECTORY_BUILDER_2D.submaps = {
  num_range_data = 90,                 -- Number of scans per submap
  grid_options_2d = {
    grid_type = "PROBABILITY_GRID",
    resolution = 0.05,                 -- Grid resolution in meters
  },
  range_data_inserter = {
    range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
    probability_grid_range_data_inserter = {
      insert_free_space = true,
      hit_probability = 0.55,
      miss_probability = 0.49,
    },
    tsdf_range_data_inserter = {
      truncation_distance = 0.3,
      maximum_weight = 10.,
      update_free_space = false,
      normal_estimation_options = {
        num_normal_samples = 4,
        sample_radius = 0.5,
      },
      project_sdf_distance_to_scan_normal = true,
      update_weight_range_exponent = 0,
      update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
      update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
    },
  },
}

-- Pose graph optimization
POSE_GRAPH = {
  optimize_every_n_nodes = 90,
  constraint_builder = {
    enabled = true,
    global_localization_min_score = 0.6,
    min_score = 0.55,
    max_constraint_distance = 15.0,
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e4,
    log_matches = false,
  },
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1.7e2,
    acceleration_exponent = 1.0,
    rotation_weight = 1.6e4,
    rotation_exponent = 1.0,
    odometry_translation_weight = 1e5,
    odometry_rotation_weight = 1e5,
    fixed_frame_pose_translation_weight = 1e5,
    fixed_frame_pose_rotation_weight = 1e5,
    log_solver_summary = false,
    use_online_imu_extrinsics_in_2d = true,
    max_num_iterations = 50,
  },
  max_num_final_iterations = 200,
  global_sampling_ratio = 0.003,
  log_residual_histograms = true,
  global_constraint_search_after_n_seconds = 10.,
}

return options
