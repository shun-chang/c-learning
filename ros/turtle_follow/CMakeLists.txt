cmake_minimum_required(VERSION 3.8)
project(turtle_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

# 编译跟随节点
add_executable(turtle_follow_node src/turtle_follow_node.cpp)
ament_target_dependencies(turtle_follow_node
  rclcpp
  turtlesim
  geometry_msgs
)

# 安装可执行文件（让 ros2 run 能找到）
install(TARGETS
  turtle_follow_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件（关键：让 ros2 launch 能找到）
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()