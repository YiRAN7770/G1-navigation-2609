cmake_minimum_required(VERSION 3.8)
project(g1_fast_lio_bringup)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mapping_frame_adapter src/mapping_frame_adapter.cpp)
target_compile_features(mapping_frame_adapter PUBLIC cxx_std_17)
target_include_directories(mapping_frame_adapter PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(mapping_frame_adapter ${PCL_LIBRARIES})
ament_target_dependencies(mapping_frame_adapter
  geometry_msgs message_filters nav_msgs pcl_conversions rclcpp sensor_msgs tf2_ros)

add_executable(prior_map_localizer src/prior_map_localizer.cpp)
target_compile_features(prior_map_localizer PUBLIC cxx_std_17)
target_include_directories(prior_map_localizer PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(prior_map_localizer ${PCL_LIBRARIES})
target_compile_definitions(prior_map_localizer PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(prior_map_localizer
  diagnostic_msgs
  geometry_msgs
  message_filters
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
)

add_executable(localization_cmd_vel_gate src/localization_cmd_vel_gate.cpp)
target_compile_features(localization_cmd_vel_gate PUBLIC cxx_std_17)
ament_target_dependencies(localization_cmd_vel_gate
  geometry_msgs
  rclcpp
  std_msgs
)

install(TARGETS
  mapping_frame_adapter
  prior_map_localizer
  localization_cmd_vel_gate
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
