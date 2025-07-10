cmake_minimum_required(VERSION 3.10)
project(kitti_cartographer_demo)

# 查找依赖项
find_package(Cartographer REQUIRED)
find_package(fmt REQUIRED)

# 添加可执行文件
add_executable(cartographer_demo
  demo.cc
)

# 链接库
target_link_libraries(cartographer_demo PRIVATE
  Cartographer::mapping
  Cartographer::sensor
  Cartographer::common
  fmt::fmt
)

# 设置C++标准
set_target_properties(kitti_demo PROPERTIES
  CXX_STANDARD 17
)