#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

using namespace cartographer;

int main() {
  // 1. 初始化MapBuilder
  mapping::proto::MapBuilderOptions map_builder_options;
  auto map_builder = mapping::CreateMapBuilder(map_builder_options);

  // 2. 加载Lua配置（示例为空配置）
  auto lua_config = std::make_unique<common::LuaParameterDictionary>(
      "return {}",  // 实际需填入有效Lua配置
      std::make_unique<common::RuntimeFileWriterResolver>());

  // 3. 添加轨迹
  const auto sensor_id = mapping::SensorId{
      mapping::SensorType::RANGE, "velodyne"};
  const int trajectory_id = map_builder->AddTrajectoryBuilder(
      {sensor_id},
      map_builder->CreateTrajectoryBuilderOptions(lua_config->GetDictionary("trajectory_builder").get()));

  // 4. 处理数据循环
  std::ifstream fin("velodyne_data.bin", std::ios::binary);
  for (int frame = 0; frame < 100; ++frame) {
    // 读取点云数据（需补充实际解析逻辑）
    sensor::TimedPointCloudData point_cloud = ReadPointCloudFromBin(fin);
    map_builder->GetTrajectoryBuilder(trajectory_id)->AddSensorData(
        sensor_id.id, point_cloud);
  }

  // 5. 保存地图
  map_builder->SerializeStateToFile(true, "kitti_map.pbstream");
  return 0;
}
