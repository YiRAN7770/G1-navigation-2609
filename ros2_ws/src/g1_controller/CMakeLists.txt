cmake_minimum_required(VERSION 3.8)
project(g1_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

# The FAST-LIO/FAR entry point uses the locally built Unitree SDK2 directly.
# The historical cmd_vel_to_g1 target below is intentionally kept unchanged
# for other launch files; only the dedicated persistent target links SDK2.
set(UNITREE_SDK2_ROOT "$ENV{UNITREE_SDK2_ROOT}" CACHE PATH "Unitree SDK2 root")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(UNITREE_SDK2_ARCH "aarch64")
else()
  set(UNITREE_SDK2_ARCH "x86_64")
endif()
set(UNITREE_SDK2_LIB_DIR "${UNITREE_SDK2_ROOT}/thirdparty/lib/${UNITREE_SDK2_ARCH}")
set(UNITREE_SDK2_STATIC "${UNITREE_SDK2_ROOT}/lib/${UNITREE_SDK2_ARCH}/libunitree_sdk2.a")
set(G1_CONTROLLER_HAS_SDK2 FALSE)
if(EXISTS "${UNITREE_SDK2_STATIC}" AND EXISTS "${UNITREE_SDK2_LIB_DIR}/libddsc.so")
  set(G1_CONTROLLER_HAS_SDK2 TRUE)
else()
  message(WARNING "Unitree SDK2 libraries not found; only legacy cmd_vel_to_g1 will be built")
endif()

# Legacy executable (used by existing non-FAST-LIO launch files).
add_executable(cmd_vel_to_g1 src/cmd_vel_to_g1.cpp)
ament_target_dependencies(cmd_vel_to_g1 rclcpp geometry_msgs)

if(G1_CONTROLLER_HAS_SDK2)
  # Persistent executable used only by system_real_robot_with_fastlio_far.launch.py.
  add_executable(cmd_vel_to_g1_persistent src/cmd_vel_to_g1_persistent.cpp)
  ament_target_dependencies(cmd_vel_to_g1_persistent rclcpp geometry_msgs)
  target_include_directories(cmd_vel_to_g1_persistent PRIVATE
    "${UNITREE_SDK2_ROOT}/include"
    "${UNITREE_SDK2_ROOT}/thirdparty/include"
    "${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx")
  target_link_directories(cmd_vel_to_g1_persistent PRIVATE "${UNITREE_SDK2_LIB_DIR}")
  target_link_libraries(cmd_vel_to_g1_persistent
    "${UNITREE_SDK2_STATIC}" ddsc ddscxx Threads::Threads)
  set_target_properties(cmd_vel_to_g1_persistent PROPERTIES
    BUILD_RPATH "${UNITREE_SDK2_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK2_LIB_DIR}")
endif()

# Install executable
install(TARGETS
  cmd_vel_to_g1
  DESTINATION lib/${PROJECT_NAME}
)
if(G1_CONTROLLER_HAS_SDK2)
  install(TARGETS cmd_vel_to_g1_persistent DESTINATION lib/${PROJECT_NAME})
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
