cmake_minimum_required(VERSION 3.8)
project(thermocator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

set(THERMOCATOR_DEPS
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  nav2_msgs
  sensor_msgs
  std_msgs
  builtin_interfaces
  geometry_msgs
)

add_executable(decision_node
    src/decision_node.cpp
)

ament_target_dependencies(decision_node ${THERMOCATOR_DEPS})
target_include_directories(decision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(decision_node PUBLIC cxx_std_17)

add_executable(thermocator
  src/thermocator.cpp
  src/map_builder.cpp
  src/thermal_grid.cpp
)
ament_target_dependencies(thermocator ${THERMOCATOR_DEPS})
target_include_directories(thermocator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(thermocator PUBLIC cxx_std_17)


add_executable(thermal_broadcaster
   src/thermal_broadcaster.cpp
)
ament_target_dependencies(thermal_broadcaster ${THERMOCATOR_DEPS})
target_include_directories(thermal_broadcaster PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(thermal_broadcaster PUBLIC cxx_std_17)

add_executable(dt_mediator
    src/dt_mediator.cpp
)
ament_target_dependencies(dt_mediator ${THERMOCATOR_DEPS})
target_compile_features(dt_mediator PUBLIC cxx_std_17)

add_executable(sync_monitor
    src/sync_monitor.cpp
)
ament_target_dependencies(sync_monitor ${THERMOCATOR_DEPS})
target_compile_features(sync_monitor PUBLIC cxx_std_17)

add_library(thermocator_display SHARED
  src/thermal_display.cpp
  include/thermocator/thermal_display.hpp
)
ament_target_dependencies(thermocator_display
  ${THERMOCATOR_DEPS}
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  pluginlib
)
target_include_directories(thermocator_display BEFORE PUBLIC
  /opt/ros/jazzy/opt/rviz_ogre_vendor/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(thermocator_display PUBLIC cxx_std_17)

target_link_libraries(thermocator_display
    Qt5::Core
    Qt5::Widgets
)

# Required for Qt MOC — RViz2 plugins use Qt signals/slots
set_property(TARGET thermocator_display PROPERTY AUTOMOC ON)

# Register plugin with pluginlib
pluginlib_export_plugin_description_file(
  rviz_common plugin_description.xml)

install(TARGETS
  thermocator
  thermocator_display
  thermal_broadcaster
  decision_node
  dt_mediator
  sync_monitor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
    FILES plugin_description.xml
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(
    DIRECTORY ../../docs/
    DESTINATION share/${PROJECT_NAME}/docs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
