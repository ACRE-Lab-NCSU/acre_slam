cmake_minimum_required(VERSION 3.8)
project(acre_cbf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(grid_map_sdf REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_executable(sdf_cbf_node src/sdf_cbf_node.cpp)

target_include_directories(sdf_cbf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(sdf_cbf_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(sdf_cbf_node
  rclcpp
  sensor_msgs
  grid_map_ros
  grid_map_msgs
  grid_map_pcl
  grid_map_sdf
  pcl_conversions
)

install(TARGETS
  sdf_cbf_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()