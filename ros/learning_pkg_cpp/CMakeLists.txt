# 最低 CMake 版本要求（ROS2 规定）
cmake_minimum_required(VERSION 3.8)

# 项目名称、版本、语言
project(learning_pkg_cpp VERSION 0.0.0)

# 启用 C++17 标准（ROS2 推荐）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包（rclcpp、std_msgs）
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 编译发布者节点
add_executable(topic_pub src/topic_pub.cpp)
ament_target_dependencies(topic_pub rclcpp std_msgs)

# 编译订阅者节点
add_executable(topic_sub src/topic_sub.cpp)
ament_target_dependencies(topic_sub rclcpp std_msgs)

# 安装可执行文件（让 ros2 run 能找到）
install(TARGETS
  topic_pub
  topic_sub
  DESTINATION lib/${PROJECT_NAME}
)

# 测试相关（默认保留）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 声明功能包类型（ament_cmake）
ament_package()