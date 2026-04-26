cmake_minimum_required(VERSION 3.5)
project(nav2_fallback_goal_checker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(fallback_goal_checker SHARED src/fallback_goal_checker.cpp)
target_include_directories(fallback_goal_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fallback_goal_checker
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS fallback_goal_checker
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(fallback_goal_checker)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  pluginlib
)

ament_package()
