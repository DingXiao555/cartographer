#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>
#include <Eigen/Core>
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"

using namespace cartographer;

// 点云读取函数实现 - 适用于KITTI格式的点云数据
sensor::TimedPointCloudData ReadPointCloudFromBin(const std::string& filename, const common::Time time) {
  std::ifstream fin(filename, std::ios::binary);
  if (!fin) {
    std::cerr << "无法打开文件: " << filename << std::endl;
    return {};
  }
  
  sensor::TimedPointCloudData data;
  data.time = time;
  
  float x, y, z, intensity;
  while (fin.read(reinterpret_cast<char*>(&x), sizeof(float)) &&
         fin.read(reinterpret_cast<char*>(&y), sizeof(float)) &&
         fin.read(reinterpret_cast<char*>(&z), sizeof(float)) &&
         fin.read(reinterpret_cast<char*>(&intensity), sizeof(float))) {
    // 在KITTI数据集中，点云是以车辆坐标系表示的
    // 我们需要将其转换为Cartographer使用的坐标系
    // KITTI: x向前，y向左，z向上
    // 转换为: x向右，y向前，z向上
    data.ranges.push_back({{y, -x, z}, 0.0f /* 相对时间 */});
  }
  
  return data;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "用法: " << argv[0] << " <数据目录路径>" << std::endl;
    std::cerr << "例如: " << argv[0] << " ~/code/data/data_object_velodyne/training/velodyne" << std::endl;
    return 1;
  }
  
  std::string data_dir = argv[1];
  std::cout << "使用数据目录: " << data_dir << std::endl;
  // 1. 获取数据目录中的所有.bin文件
  std::vector<std::string> bin_files;
  std::string command = "ls " + data_dir + "/*.bin | sort";
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) {
    std::cerr << "无法执行命令: " << command << std::endl;
    return 1;
  }
  
  char buffer[1024];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    std::string file(buffer);
    if (!file.empty() && file.back() == '\n') {
      file.pop_back();  // 移除换行符
    }
    bin_files.push_back(file);
  }
  pclose(pipe);
  
  std::cout << "找到 " << bin_files.size() << " 个点云文件" << std::endl;
  if (bin_files.empty()) {
    std::cerr << "未找到点云文件，请检查数据目录路径" << std::endl;
    return 1;
  }
  
  // 2. 初始化MapBuilder（内部会创建线程池）
  mapping::proto::MapBuilderOptions map_builder_options;
  // 设置线程池大小
  map_builder_options.set_num_background_threads(4);
  // 设置使用3D轨迹构建器（因为KITTI数据集是3D点云）
  map_builder_options.set_use_trajectory_builder_3d(true);
  map_builder_options.set_use_trajectory_builder_2d(false);
  auto map_builder = mapping::CreateMapBuilder(map_builder_options);
  
  // 3. 加载Lua配置
  auto lua_config = std::make_unique<common::LuaParameterDictionary>(
    R"(
    return {
      trajectory_builder = {
        collate_fixed_frame = false,
        collate_landmarks = false,
        -- use_3d参数现在在MapBuilderOptions中设置
        trajectory_builder_2d = {
          -- 添加一些默认配置，即使我们不使用它
          use_imu_data = false,
          min_range = 0.1,
          max_range = 100.0,
          missing_data_ray_length = 5.0,
          num_accumulated_range_data = 1,
          voxel_filter_size = 0.025,
          min_z = -0.8,
          max_z = 2.0,
          use_online_correlative_scan_matching = false,
          real_time_correlative_scan_matcher = {
            linear_search_window = 0.1,
            angular_search_window = math.rad(20.),
            translation_delta_cost_weight = 1e-1,
            rotation_delta_cost_weight = 1e-1,
          },
          adaptive_voxel_filter = {
            max_length = 0.5,
            min_num_points = 200,
            max_range = 50.,
          },
          loop_closure_adaptive_voxel_filter = {
            max_length = 0.9,
            min_num_points = 100,
            max_range = 50.,
          },
          motion_filter = {
            max_time_seconds = 5.,
            max_distance_meters = 0.2,
            max_angle_radians = math.rad(1.),
          },
          imu_gravity_time_constant = 10.,
          pose_extrapolator = {
            use_imu_based = false,
            constant_velocity = {
              pose_queue_duration = 0.001,
              imu_gravity_time_constant = 10.0,
            },
            imu_based = {
              pose_queue_duration = 5.,
              gravity_constant = 9.806,
              pose_translation_weight = 1.,
              pose_rotation_weight = 1.,
              imu_acceleration_weight = 1.,
              imu_rotation_weight = 1.,
              odometry_translation_weight = 1.,
              odometry_rotation_weight = 1.,
              solver_options = {
                use_nonmonotonic_steps = false,
                max_num_iterations = 10,
                num_threads = 1,
              },
            },
          },
          ceres_scan_matcher = {
            occupied_space_weight = 1.0,
            translation_weight = 10.0,
            rotation_weight = 40.0,
            ceres_solver_options = {
              use_nonmonotonic_steps = false,
              max_num_iterations = 20,
              num_threads = 1,
            },
          },
          submaps = {
            num_range_data = 90,
            grid_options_2d = {
              grid_type = "PROBABILITY_GRID",
              resolution = 0.05,
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
                maximum_weight = 10.0,
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
          },
        },
        trajectory_builder_3d = {
          min_range = 1.,
          max_range = 60.,
          num_accumulated_range_data = 1,
          voxel_filter_size = 0.15,
          
          high_resolution_adaptive_voxel_filter = {
            max_length = 2.,
            min_num_points = 150,
            max_range = 15.,
          },
          
          low_resolution_adaptive_voxel_filter = {
            max_length = 4.,
            min_num_points = 200,
            max_range = 60.,
          },
          
          use_online_correlative_scan_matching = false,
          real_time_correlative_scan_matcher = {
            linear_search_window = 0.15,
            angular_search_window = math.rad(1.),
            translation_delta_cost_weight = 1e-1,
            rotation_delta_cost_weight = 1e-1,
          },
          
          ceres_scan_matcher = {
            occupied_space_weight_0 = 1.,
            occupied_space_weight_1 = 6.,
            translation_weight = 5.,
            rotation_weight = 4e2,
            only_optimize_yaw = false,
            ceres_solver_options = {
              use_nonmonotonic_steps = false,
              max_num_iterations = 12,
              num_threads = 1,
            },
          },
          
          motion_filter = {
            max_time_seconds = 0.5,
            max_distance_meters = 0.1,
            max_angle_radians = 0.004,
          },
          
          rotational_histogram_size = 120,
          
          imu_gravity_time_constant = 10.,
          pose_extrapolator = {
            use_imu_based = false,
            constant_velocity = {
              imu_gravity_time_constant = 10.,
              pose_queue_duration = 0.001,
            },
            imu_based = {
              pose_queue_duration = 5.,
              gravity_constant = 9.806,
              pose_translation_weight = 1.,
              pose_rotation_weight = 1.,
              imu_acceleration_weight = 1.,
              imu_rotation_weight = 1.,
              odometry_translation_weight = 1.,
              odometry_rotation_weight = 1.,
              solver_options = {
                use_nonmonotonic_steps = false,
                max_num_iterations = 10,
                num_threads = 1,
              },
            },
          },
          
          submaps = {
            high_resolution = 0.10,
            high_resolution_max_range = 20.,
            low_resolution = 0.45,
            num_range_data = 160,
            range_data_inserter = {
              hit_probability = 0.55,
              miss_probability = 0.49,
              num_free_space_voxels = 2,
              intensity_threshold = 0.5,
            },
          },
          
          use_intensities = false,
        },
      },
      pose_graph = {
        optimize_every_n_nodes = 50,
        constraint_builder = {
          sampling_ratio = 0.3,
          max_constraint_distance = 15,
        },
      },
    }
  )", 
    std::make_unique<common::ConfigurationFileResolver>(std::vector<std::string>{})
  );

  // 4. 添加轨迹
  // 创建传感器ID
  const std::string sensor_id = "velodyne";
  
  // 创建传感器ID集合
  std::set<mapping::TrajectoryBuilderInterface::SensorId> sensor_ids;
  sensor_ids.insert(mapping::TrajectoryBuilderInterface::SensorId{
      mapping::TrajectoryBuilderInterface::SensorId::SensorType::RANGE, sensor_id});
  
  // 获取配置选项
  auto trajectory_builder_options = mapping::CreateTrajectoryBuilderOptions(
      lua_config->GetDictionary("trajectory_builder").get());
  
  // 确保imu_gravity_time_constant参数被设置
  if (!trajectory_builder_options.has_trajectory_builder_3d_options()) {
    trajectory_builder_options.mutable_trajectory_builder_3d_options();
  }
  if (!trajectory_builder_options.trajectory_builder_3d_options().has_pose_extrapolator_options()) {
    trajectory_builder_options.mutable_trajectory_builder_3d_options()
        ->mutable_pose_extrapolator_options();
  }
  if (!trajectory_builder_options.trajectory_builder_3d_options()
           .pose_extrapolator_options()
           .has_constant_velocity()) {
    trajectory_builder_options.mutable_trajectory_builder_3d_options()
        ->mutable_pose_extrapolator_options()
        ->mutable_constant_velocity();
  }
  trajectory_builder_options.mutable_trajectory_builder_3d_options()
      ->mutable_pose_extrapolator_options()
      ->mutable_constant_velocity()
      ->set_imu_gravity_time_constant(10.0);
  // 确保imu_gravity_time_constant参数被设置
  trajectory_builder_options.mutable_trajectory_builder_3d_options()
      ->mutable_pose_extrapolator_options()
      ->mutable_constant_velocity()
      ->set_imu_gravity_time_constant(10.0);
  
  const int trajectory_id = map_builder->AddTrajectoryBuilder(
      sensor_ids,
      trajectory_builder_options,
      [](int /* trajectory_id */,
         common::Time /* time */,
         transform::Rigid3d /* local_pose */,
         sensor::RangeData /* range_data */,
         std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult> /* insertion_result */) {
        // 这是一个空回调函数，不做任何事情
      });
  
  std::cout << "创建了轨迹ID: " << trajectory_id << std::endl;

  // 5. 处理数据
  const int max_files_to_process = std::min(static_cast<int>(bin_files.size()), 100);  // 限制处理的文件数量
  std::cout << "将处理前 " << max_files_to_process << " 个点云文件" << std::endl;
  
  for (int i = 0; i < max_files_to_process; ++i) {
    // 为每个点云分配时间戳，以10Hz的频率
    const common::Time time = common::Time() + common::FromSeconds(i * 0.1);
    
    std::cout << "处理文件 " << (i + 1) << "/" << max_files_to_process 
              << ": " << bin_files[i] << std::endl;
    
    sensor::TimedPointCloudData point_cloud = ReadPointCloudFromBin(bin_files[i], time);
    if (point_cloud.ranges.empty()) {
      std::cerr << "警告: 文件 " << bin_files[i] << " 未包含有效点云数据，跳过" << std::endl;
      continue;
    }
    
    map_builder->GetTrajectoryBuilder(trajectory_id)->AddSensorData(
        sensor_id, point_cloud);
  }

  // 6. 完成轨迹并保存地图
  map_builder->FinishTrajectory(trajectory_id);
  std::cout << "正在优化地图..." << std::endl;
  map_builder->pose_graph()->RunFinalOptimization();
  
  const std::string map_filename = "kitti_map.pbstream";
  std::cout << "保存地图到 " << map_filename << std::endl;
  map_builder->SerializeStateToFile(true, map_filename);
  
  std::cout << "建图完成!" << std::endl;
  return 0;
}