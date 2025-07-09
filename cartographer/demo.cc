#include <iostream>
#include <fstream>
#include <memory>
#include <Eigen/Core>
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/common/lua_parameter_dictionary.h"


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
      "return {}",
      std::make_unique<common::file::RuntimeFileWriterResolver>());  // 修正命名空间

  // 3. 添加轨迹
  const auto sensor_id = mapping::proto::SensorId{  // 修正命名空间
      mapping::proto::SensorType::RANGE, "velodyne"};
  const int trajectory_id = map_builder->AddTrajectoryBuilder(
      {sensor_id},
      map_builder->trajectory_builder_options());  // 修正方法名

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
