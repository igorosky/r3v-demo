cmake_minimum_required(VERSION 3.16)
project(r3v_viewer CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

get_filename_component(R3V_REPOSITORY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(R3V_RENDERER_DIR "${R3V_REPOSITORY_DIR}/r3v-renderer" CACHE PATH
  "r3v-renderer source checkout")
set(R3V_VOXELIZER_DIR "${R3V_REPOSITORY_DIR}/r3v-voxelizer" CACHE PATH
  "r3v-voxelizer source checkout")

include(cmake/r3v_rust.cmake)
r3v_add_rust_library(renderer "${R3V_RENDERER_DIR}" r3v_renderer "c,cpp,multi-threaded")
r3v_add_rust_library(voxelizer "${R3V_VOXELIZER_DIR}" r3v_voxelizer "c,cpp,multi_threaded,wgpu")

add_executable(r3v_viewer src/viewer.cpp)
target_compile_features(r3v_viewer PRIVATE cxx_std_17)
target_link_libraries(r3v_viewer
  r3v::renderer
  r3v::voxelizer
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros)
add_dependencies(r3v_viewer renderer_cargo voxelizer_cargo)
set_target_properties(r3v_viewer PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(TARGETS r3v_viewer DESTINATION lib/${PROJECT_NAME})
install(FILES "${renderer_LIBRARY}" "${voxelizer_LIBRARY}" DESTINATION lib)

ament_package()
