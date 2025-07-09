#include <iostream>
#include <fstream>
#include <memory>
#include <Eigen/Core>
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/configuration_file_resolver.h"  // 新增头文件


using namespace cartographer;

// 点云读取函数实现
sensor::TimedPointCloudData ReadPointCloudFromBin(std::ifstream& fin) {
  sensor::TimedPointCloudData data;
  float x, y, z, intensity;
  while (fin.read(reinterpret_cast<char*>(&x), sizeof(float)) &&
         fin.read(reinterpret_cast<char*>(&y), sizeof(float)) &&
         fin.read(reinterpret_cast<char*>(&z), sizeof(float)) &&
         fin.read(reinterpret_cast<char*>(&intensity), sizeof(float))) {
    data.ranges.push_back({Eigen::Vector3f(x, y, z), 0.0f /* time */});
  }
  return data;
}

int main() {
  // 1. 初始化MapBuilder
  mapping::proto::MapBuilderOptions map_builder_options;
  auto map_builder = mapping::CreateMapBuilder(map_builder_options);

  // 2. 加载Lua配置
  auto lua_config = std::make_unique<common::LuaParameterDictionary>(
    R"(
    return {
      trajectory_builder = {
        use_3d = true,
        trajectory_builder_3d = {
          num_accumulated_range_data = 1,
          voxel_filter_size = 0.15,
          submaps = {
            high_resolution = 0.25,
            low_resolution = 0.6,
            num_range_data = 100,
          },
          ceres_pose_gravity_alignment = false,
          motion_filter = {
            max_time_seconds = 30,
            max_distance_meters = 0.3,
            max_angle_radians = math.rad(2),
          },
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
    std::make_unique<common::ConfigurationFileResolver>()
);

  // 3. 添加轨迹
  const auto sensor_id = mapping::proto::SensorId{  // 修正命名空间
      mapping::proto::SensorId::RANGE, "velodyne"};
  const int trajectory_id = map_builder->AddTrajectoryBuilder(
      {sensor_id},
      map_builder->GetAllTrajectoryBuilderOptions());  // 修正方法名

  // 4. 处理数据
  std::ifstream fin("velodyne_data.bin", std::ios::binary);
  for (int frame = 0; frame < 100; ++frame) {
    sensor::TimedPointCloudData point_cloud = ReadPointCloudFromBin(fin);
    map_builder->GetTrajectoryBuilder(trajectory_id)->AddSensorData(
        sensor_id.id(), point_cloud);  // 使用id()方法
  }

  // 5. 保存地图
  map_builder->SerializeStateToFile(true, "kitti_map.pbstream");
  return 0;
}
