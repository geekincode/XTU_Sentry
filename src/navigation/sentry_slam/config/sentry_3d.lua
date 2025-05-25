include "map_builder.lua"
include "trajectory_builder.lua"
-- include "trajectory_builder_3d.lua"

-- 明确禁用2D轨迹构建器（防止冲突）
TRAJECTORY_BUILDER_2D = nil

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- 传感器配置
  num_point_clouds = 1,        -- 使用一个点云输入
  num_laser_scans = 0,         -- 禁用2D激光扫描
  use_odometry = true,         -- 启用里程计（如差分/IMU融合）
  use_imu_data = true,         -- 启用IMU数据用于姿态估计

  -- 坐标系设置（需与URDF一致）
  map_frame = "map",
  tracking_frame = "imu_link", -- 推荐：使用IMU坐标系作为跟踪基准
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,

  -- 数据采样率
  rangefinder_sampling_ratio = 1.0,  -- 点云全采样
  imu_sampling_ratio = 1.0           -- IMU全采样
}

-- 启用3D轨迹构建器
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4  -- 多线程加速建图

-- 3D轨迹构建参数
TRAJECTORY_BUILDER_3D = {
  min_range = 0.5,                    -- 过滤掉太近的数据（Mid360有效范围约0.1m~50m）
  max_range = 50.0,                   -- Mid360最大探测距离
  num_accumulated_range_data = 1,     -- 实时性优先，不积累多帧
  voxel_filter_size = 0.05,           -- 更精细的点云滤波，提升建图质量
  high_resolution_adaptive_voxel_filter = {
    max_length = 2.0,
    max_range = 15.0,
    min_num_points = 150
  },
  low_resolution_adaptive_voxel_filter = {
    max_length = 4.0,
    max_range = 60.0,
    min_num_points = 200
  },
  motion_filter = {
    max_time_seconds = 0.5,            -- 时间运动滤波阈值
    max_distance_meters = 0.05,        -- 小位移下才触发新子图
    max_angle_radians = math.rad(0.5)   -- 小角度变化也触发新子图
  },
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.3,         -- 增大搜索窗口提高匹配鲁棒性
    angular_search_window = math.rad(5), -- 提高旋转容错
    translation_delta_cost_weight = 10.0,
    rotation_delta_cost_weight = 10.0
  }
}

-- 图优化参数
POSE_GRAPH = {
  optimize_every_n_nodes = 90,               -- 每90个节点优化一次
  constraint_builder = {
    min_score = 0.65,                        -- 提高回环匹配门槛，避免误匹配
    global_localization_min_score = 0.70,    -- 全局定位最低分数
    loop_closure_translation_weight = 10000, -- 回环平移权重
    loop_closure_rotation_weight = 1e5       -- 回环旋转权重更高，强调方向一致性
  },
  matcher_translation_weight = 10000,         -- 匹配权重调优
  matcher_rotation_weight = 1e5,
  optimization_problem = {
    use_online_imu_extrinsics_in_3d = true, -- 在线校准IMU和LiDAR外参
    huber_scale = 1.0,
    acceleration_weight = 1000.0,
    rotation_weight = 1e4
  }
}

return options