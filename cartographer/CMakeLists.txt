cmake_minimum_required(VERSION 3.10)
project(kitti_cartographer_demo)

# 设置Cartographer库的配置文件路径
set(Cartographer_DIR /usr/local/share/cartographer)

# 查找依赖项
find_package(Cartographer REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)

# 添加可执行文件
add_executable(cartographer_demo
  demo.cc
)

# 链接库
target_link_libraries(cartographer_demo PRIVATE
  cartographer
  Boost::iostreams
)

# 设置C++标准
set_target_properties(cartographer_demo PROPERTIES
  CXX_STANDARD 17
)