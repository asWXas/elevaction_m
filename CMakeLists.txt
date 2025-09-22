cmake_minimum_required(VERSION 3.8)
project(online)

# 设置推荐的 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 添加编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------- 1. 查找依赖包 --------------------
# 查找所有需要的 ROS 2 和第三方包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
# 为 OctoMap 生成器添加 tf2_ros 依赖
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)


# -------------------- 2. 定义可执行文件及其依赖 --------------------
# -- 可执行文件 1: 点云预处理器 --
add_executable(pointcloud_processor src/as.cpp)
add_executable(tf_broadcaster src/tf.cpp)
# 使用 ament_target_dependencies 自动处理包含和链接
ament_target_dependencies(pointcloud_processor
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
)
ament_target_dependencies(tf_broadcaster
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

# -- 可执行文件 2: OctoMap 地图生成器 --
# # 建议给可执行文件一个更具描述性的名字，例如 octomap_generator
# add_executable(octomap_generator src/sa.cpp)
# # 自动处理所有相关依赖
# ament_target_dependencies(octomap_generator
#   rclcpp
#   sensor_msgs
#   nav_msgs
#   pcl_conversions
#   octomap
#   octomap_msgs
#   tf2_ros
# )


# -------------------- 3. 安装可执行文件 (非常重要!) --------------------
# 这个部分确保 `ros2 run` 可以找到你的节点
install(
  TARGETS
    pointcloud_processor
    # octomap_generator
  DESTINATION lib/${PROJECT_NAME}
)


# -------------------- 4. 测试 (可选) --------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# -------------------- 5. 完成包的定义 --------------------
ament_package()