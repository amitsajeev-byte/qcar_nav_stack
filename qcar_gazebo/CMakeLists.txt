cmake_minimum_required(VERSION 3.8)
project(qcar_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

# --- custom MPPI critic plugin (src/critics/) ---
# xtensor/xsimd defines below MUST match nav2_mppi_controller's own CMakeLists.txt exactly
# (github.com/ros-navigation/navigation2, humble branch) - xtensor's container layout/alignment
# changes depending on whether XTENSOR_USE_XSIMD is defined at compile time, so a mismatch here
# is a silent ABI mismatch: xt::xtensor<float,N> objects built by nav2's precompiled binary and
# by this plugin would disagree on memory layout despite being "the same" C++ type, corrupting
# memory on any vectorized operation (e.g. xt::pow) crossing the plugin boundary via CriticData.
# Without these defines, the plugin loads and runs fine right up until any xtensor math heavier
# than a scalar += touches data.costs, then segfaults the whole nav2_container process.
add_definitions(-DXTENSOR_ENABLE_XSIMD)
add_definitions(-DXTENSOR_USE_XSIMD)

find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)

add_library(qcar_critics SHARED
  src/critics/early_commit_critic.cpp
)
target_include_directories(qcar_critics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(qcar_critics
  nav2_mppi_controller
  nav2_costmap_2d
  rclcpp
  rclcpp_lifecycle
  pluginlib
  xtensor
)
target_link_libraries(qcar_critics xtensor::optimize xtensor::use_xsimd)
pluginlib_export_plugin_description_file(nav2_mppi_controller qcar_critics.xml)

install(TARGETS qcar_critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
# --- end custom MPPI critic plugin ---

# --- custom smoother_server plugin (src/smoothers/) ---
find_package(nav2_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(qcar_smoothers SHARED
  src/smoothers/cusp_straightener_smoother.cpp
)
target_include_directories(qcar_smoothers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(qcar_smoothers
  nav2_core
  nav2_costmap_2d
  rclcpp
  rclcpp_lifecycle
  pluginlib
  tf2
  tf2_geometry_msgs
)
pluginlib_export_plugin_description_file(nav2_core qcar_smoothers.xml)

install(TARGETS qcar_smoothers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# --- end custom smoother_server plugin ---

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY urdf launch worlds models config rviz scripts maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/qcar_teleop_twist.py
  scripts/qcar_relay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(qcar_critics qcar_smoothers)
ament_export_dependencies(nav2_mppi_controller nav2_costmap_2d nav2_core rclcpp rclcpp_lifecycle pluginlib xtensor tf2 tf2_geometry_msgs)

ament_package()
