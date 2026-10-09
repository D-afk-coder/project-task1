################################################################################
# CMakeLists.txt
# Build script for turtlebot3_gazebo. This is the upstream ROBOTIS file
# (Apache License 2.0) with three changes for MTRX3760 Project 1: the
# turtlebot3_drive executable is built from the five source files of the right
# wall follower and also links std_msgs; the Gazebo obstacle plugins are only
# built when the Gazebo development packages are present; and unit tests are
# added, built only when ament_cmake_gtest is installed.
################################################################################
cmake_minimum_required(VERSION 3.8)
project(turtlebot3_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Only needed for the obstacle plugins used by the DQN stage worlds.
find_package(ros_gz_sim QUIET)
find_package(gz_math_vendor QUIET)
find_package(gz-math QUIET)
find_package(gz_sim_vendor QUIET)
find_package(gz-sim QUIET)
find_package(gz_plugin_vendor QUIET)
find_package(gz-plugin QUIET)

################################################################################
# Build
################################################################################
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${GZ_SIM_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${gz_sim_vendor_INCLUDE_DIRS}
  ${GZ_SIM_INCLUDE_DIRS}
)

# Right wall follower (MTRX3760 Project 1)
set(EXEC_NAME "turtlebot3_drive")
set(WALL_FOLLOWER_SOURCES
  src/scan_interpreter.cpp
  src/drive_states.cpp
  src/wall_follower_controller.cpp
  src/wall_follower_config.cpp
)

add_executable(${EXEC_NAME} src/turtlebot3_drive.cpp ${WALL_FOLLOWER_SOURCES})
target_link_libraries(${EXEC_NAME}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

# Gazebo obstacle plugins (upstream, unchanged)
set(OBSTACLE_LIBS
  obstacles
  obstacle1
  obstacle2
)

if(TARGET gz-sim::gz-sim AND TARGET gz-math::gz-math)
  foreach(lib ${OBSTACLE_LIBS})
    add_library(${lib} SHARED src/${lib}.cpp)
    target_link_libraries(${lib}
      PUBLIC
      gz-sim::gz-sim
      gz-math::gz-math
    )
  endforeach()
  install(TARGETS ${OBSTACLE_LIBS} DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "Gazebo development packages not found: skipping obstacle plugins")
endif()

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch models params rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Tests (MTRX3760 Project 1)
################################################################################
# Optional: skipped (with a message) if ament_cmake_gtest is not installed,
# so a missing test package never stops the robot code from building.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    foreach(test_name test_scan_interpreter test_controller test_maze_sim)
      ament_add_gtest(${test_name} test/${test_name}.cpp ${WALL_FOLLOWER_SOURCES}
        TIMEOUT 300)
      target_include_directories(${test_name} PRIVATE test)
    endforeach()
  else()
    message(STATUS "ament_cmake_gtest not found: skipping wall follower tests")
  endif()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)
ament_package()
