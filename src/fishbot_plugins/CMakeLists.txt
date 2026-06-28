cmake_minimum_required(VERSION 3.8)
project(fishbot_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(
  buoyancy_plugin SHARED
  src/buoyancy_plugin.cpp
)

target_link_libraries(
  buoyancy_plugin
  ${GAZEBO_LIBRARIES}
)

add_library(drag_plugin SHARED
  src/drag_plugin.cpp
)

target_link_libraries(drag_plugin
  ${GAZEBO_LIBRARIES}
)

add_library(thrust_plugin SHARED
  src/thrust_plugin.cpp
)

target_link_libraries(thrust_plugin
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS
    buoyancy_plugin
    drag_plugin
    thrust_plugin
  LIBRARY DESTINATION lib
)

ament_package()