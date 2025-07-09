include "pose_graph.lua"
include "trajectory_builder_3d.lua"

MAP_BUILDER = {
  use_trajectory_builder_3d = true,
  num_background_threads = 4,
}

TRAJECTORY_BUILDER_3D = {
  submaps = {
    num_range_data = 90,              -- 每个子图包含90帧点云
    range_data_inserter = {
      probability_grid_range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
    },
  },
  use_imu_data = false,               -- 关闭IMU（因KITTI测试集无IMU）
  imu_gravity_time_constant = 10.0,
  rotational_histogram_size = 120,
}

POSE_GRAPH = {
  optimize_every_n_nodes = 50,        -- 每50帧优化一次
  constraint_builder = {
    sampling_ratio = 0.3,
    max_constraint_distance = 15.,
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
  },
}

return {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER_3D,
  pose_graph = POSE_GRAPH,
}
