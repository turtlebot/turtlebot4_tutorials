cmake_minimum_required(VERSION 3.8)
project(turtlebot4_cpp_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(irobot_create_msgs REQUIRED)

add_executable(turtlebot4_first_cpp_node src/turtlebot4_first_cpp_node.cpp)

ament_target_dependencies(turtlebot4_first_cpp_node rclcpp irobot_create_msgs)

install(TARGETS turtlebot4_first_cpp_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
