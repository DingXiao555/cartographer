#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Core>
#include "cartographer/mapping/map_builder.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

using namespace cartographer;

// 读取单个bin文件
sensor::TimedPointCloudData ReadBinFile(const std::string& path, common::Time time) {
  std::ifstream file(path, std::ios::binary);
  std::vector<float> buffer(4);
  sensor::TimedPointCloud points;

  while (file.read(reinterpret_cast<char*>(buffer.data()), 4 * sizeof(float))) {
    points.emplace_back(buffer[0], buffer[1], buffer[2]);  // 忽略反射强度
  }

  return sensor::TimedPointCloudData{
      time,                   // 时间戳（需按帧递增）
      Eigen::Vector3f::Zero(),// 坐标原点设为(0,0,0)
      std::move(points)
  };
}

int main() {
  // 初始化建图器
  auto map_builder = mapping::CreateMapBuilder();

  // 加载配置
  auto lua_config = common::LuaParameterDictionary::NonNativeHandle(
      "../configuration_files/kitti_3d.lua",
      {}  // 无额外参数
  );

  // 创建轨迹
  const int trajectory_id = map_builder->AddTrajectoryBuilder(
      {mapping::TrajectoryBuilderInterface::SensorId{
          mapping::TrajectoryBuilderInterface::SensorType::RANGE, "velodyne"}},
      lua_config.GetDictionary("trajectory_builder").get()
  );

  // 处理所有bin文件
  common::Time current_time = common::FromUniversal(0);  // 时间起点
  const double delta_time = 0.1;                         // 假设10Hz采样率

  for (int frame = 0; ; ++frame) {
    const std::string bin_path = fmt::format(
        "../kitti_data/testing/velodyne/{:06d}.bin", frame);

    if (!std::filesystem::exists(bin_path)) break;  // 文件不存在时退出

    auto cloud_data = ReadBinFile(bin_path, current_time);
    map_builder->GetTrajectoryBuilder(trajectory_id)
        ->AddSensorData("velodyne", cloud_data);

    current_time += common::FromSeconds(delta_time);
  }

  // 保存地图
  map_builder->SerializeStateToFile(true, "../build/kitti_map.pbstream");
  std::cout << "建图完成！结果保存至 build/kitti_map.pbstream" << std::endl;
  return 0;
}
