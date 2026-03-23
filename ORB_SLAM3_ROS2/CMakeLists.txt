cmake_minimum_required(VERSION 3.5)
project(orbslam3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules
)

find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

# Includes and libs come from FindORB_SLAM3.cmake (ORB_SLAM3_INCLUDE_DIRS, ORB_SLAM3_LIBRARIES).

# ================== mono ==================
add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)

target_link_libraries(mono
  ${OpenCV_LIBS}
)

target_include_directories(mono PUBLIC
  include
  ${ORB_SLAM3_INCLUDE_DIRS}
)

ament_target_dependencies(mono
  rclcpp sensor_msgs cv_bridge std_srvs ORB_SLAM3 Pangolin geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs visualization_msgs
)

# ============ mono-inertial ================
add_executable(mono-inertial
  src/monocular-inertial/mono-inertial.cpp
  src/monocular-inertial/mono-inertial-node.cpp
)

target_link_libraries(mono-inertial
  ${OpenCV_LIBS}
)

target_include_directories(mono-inertial PUBLIC
  include
  ${ORB_SLAM3_INCLUDE_DIRS}
)

ament_target_dependencies(mono-inertial
  rclcpp sensor_msgs cv_bridge std_srvs ORB_SLAM3 Pangolin geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs visualization_msgs
)

# ================== rgbd ==================
add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)

target_link_libraries(rgbd
  ${OpenCV_LIBS}
)

target_include_directories(rgbd PUBLIC
  include
  ${ORB_SLAM3_INCLUDE_DIRS}
)

ament_target_dependencies(rgbd
  rclcpp sensor_msgs cv_bridge message_filters std_srvs ORB_SLAM3 Pangolin
)

# ================== stereo ==================
add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)

target_link_libraries(stereo
  ${OpenCV_LIBS}
)

target_include_directories(stereo PUBLIC
  include
  ${ORB_SLAM3_INCLUDE_DIRS}
)

ament_target_dependencies(stereo
  rclcpp sensor_msgs cv_bridge message_filters std_srvs ORB_SLAM3 Pangolin
)

# ============ stereo-inertial ===============
add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)

target_link_libraries(stereo-inertial
  ${OpenCV_LIBS}
)

target_include_directories(stereo-inertial PUBLIC
  include
  ${ORB_SLAM3_INCLUDE_DIRS}
)

ament_target_dependencies(stereo-inertial
  rclcpp sensor_msgs cv_bridge std_srvs ORB_SLAM3 Pangolin
)

# =============== install ====================
install(
  TARGETS mono mono-inertial rgbd stereo stereo-inertial
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

