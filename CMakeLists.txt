cmake_minimum_required(VERSION 3.8)
project(ekf_localization)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

# Include directories
include_directories(include)

# Create ekf_core library
add_library(ekf_core 
  src/ekf_core.cpp
  src/utils.cpp
)
target_link_libraries(ekf_core glog::glog)
ament_target_dependencies(ekf_core Eigen3)
target_include_directories(ekf_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Create ekf_node executable
add_executable(ekf_node src/ekf_node.cpp)

ament_target_dependencies(ekf_node 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  geometry_msgs
  tf2
  tf2_ros
)

target_include_directories(ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ekf_node ekf_core glog::glog)

# Install targets
install(TARGETS ekf_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Lint tests
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests (commented out until test files are created)
  # ament_add_gtest(test_ekf_core test/test_ekf_core.cpp)
  # target_link_libraries(test_ekf_core ekf_core)
endif()

ament_package()