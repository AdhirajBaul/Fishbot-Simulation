cmake_minimum_required(VERSION 3.8)
project(fishbot_plugins)

# ============================================================
# Compiler settings
# ============================================================

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

# ============================================================
# Gazebo configuration
# ============================================================

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(
  CMAKE_CXX_FLAGS
  "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}"
)

# ============================================================
# Buoyancy plugin
# ============================================================

add_library(
  buoyancy_plugin
  SHARED
  src/buoyancy_plugin.cpp
)

target_link_libraries(
  buoyancy_plugin
  ${GAZEBO_LIBRARIES}
)

# ============================================================
# Drag plugin
# ============================================================

add_library(
  drag_plugin
  SHARED
  src/drag_plugin.cpp
)

target_link_libraries(
  drag_plugin
  ${GAZEBO_LIBRARIES}
)

# ============================================================
# Tail thrust and steering plugin
# ============================================================

add_library(
  thrust_plugin
  SHARED
  src/thrust_plugin.cpp
)

target_link_libraries(
  thrust_plugin
  ${GAZEBO_LIBRARIES}
)

# ============================================================
# Pectoral-fin hydrodynamics plugin
# ============================================================

add_library(
  fin_hydrodynamics_plugin
  SHARED
  src/fin_hydrodynamics_plugin.cpp
)

target_link_libraries(
  fin_hydrodynamics_plugin
  ${GAZEBO_LIBRARIES}
)

# ============================================================
# Gazebo keyboard GUI plugin
# ============================================================

add_library(
  fishbot_keyboard_gui
  SHARED
  src/fishbot_keyboard_gui.cpp
)

target_include_directories(
  fishbot_keyboard_gui
  PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)

ament_target_dependencies(
  fishbot_keyboard_gui
  rclcpp
  std_msgs
)

target_link_libraries(
  fishbot_keyboard_gui
  ${GAZEBO_LIBRARIES}
  Qt5::Widgets
)

# ============================================================
# Installation
# ============================================================

install(
  TARGETS
    buoyancy_plugin
    drag_plugin
    thrust_plugin
    fin_hydrodynamics_plugin
    fishbot_keyboard_gui

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================
# Export dependencies
# ============================================================

ament_export_dependencies(
  gazebo
  rclcpp
  std_msgs
)

ament_package()