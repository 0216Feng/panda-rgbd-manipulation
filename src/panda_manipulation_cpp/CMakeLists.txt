cmake_minimum_required(VERSION 3.16)
project(panda_manipulation_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(trajectory_quality src/trajectory_quality.cpp)
target_include_directories(
  trajectory_quality
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(trajectory_quality trajectory_msgs)

add_executable(trajectory_metrics_node src/trajectory_metrics_node.cpp)
target_link_libraries(trajectory_metrics_node trajectory_quality)
ament_target_dependencies(
  trajectory_metrics_node
  moveit_msgs
  rclcpp
  std_msgs
  trajectory_msgs
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(trajectory_quality PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(trajectory_metrics_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS trajectory_quality trajectory_metrics_node
  EXPORT export_trajectory_quality
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory_quality test/test_trajectory_quality.cpp)
  target_link_libraries(test_trajectory_quality trajectory_quality)
endif()

ament_export_targets(export_trajectory_quality HAS_LIBRARY_TARGET)
ament_export_dependencies(trajectory_msgs)
ament_package()
