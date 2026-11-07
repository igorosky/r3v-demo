cmake_minimum_required(VERSION 3.8)
project(r3v_sim NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/oscillator.py scripts/patrol.py DESTINATION lib/${PROJECT_NAME})

ament_package()
