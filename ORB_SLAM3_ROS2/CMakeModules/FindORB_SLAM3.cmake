# Find ORB_SLAM3 (non-CMake-config build: shared lib under source tree).
#
# Resolution order for ORB_SLAM3_ROOT_DIR:
#   1) CMake cache / -DORB_SLAM3_ROOT_DIR
#   2) Environment variable ORB_SLAM3_ROOT_DIR
#   3) Sibling directory: parent(ORB_SLAM3_ROS2)/ORB_SLAM3 (monorepo layout)
#
# Defines: ORB_SLAM3_FOUND, ORB_SLAM3_LIBRARIES, ORB_SLAM3_INCLUDE_DIRS

if(NOT ORB_SLAM3_ROOT_DIR)
  if(DEFINED ENV{ORB_SLAM3_ROOT_DIR})
    string(STRIP "$ENV{ORB_SLAM3_ROOT_DIR}" _orb_env_root)
    if(NOT _orb_env_root STREQUAL "")
      get_filename_component(ORB_SLAM3_ROOT_DIR "${_orb_env_root}" ABSOLUTE)
    endif()
  endif()
endif()

if(NOT ORB_SLAM3_ROOT_DIR)
  get_filename_component(_ros2_pkg_dir "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
  get_filename_component(_default_root "${_ros2_pkg_dir}/../ORB_SLAM3" ABSOLUTE)
  if(EXISTS "${_default_root}/include/System.h")
    set(ORB_SLAM3_ROOT_DIR "${_default_root}")
  endif()
endif()

if(NOT ORB_SLAM3_ROOT_DIR)
  message(FATAL_ERROR
    "ORB_SLAM3 not found. Set ORB_SLAM3_ROOT_DIR:\n"
    "  export ORB_SLAM3_ROOT_DIR=/path/to/ORB_SLAM3\n"
    "or pass -DORB_SLAM3_ROOT_DIR=/path/to/ORB_SLAM3 to cmake/colcon.\n"
    "Or place the core library next to this package: .../ORB_SLAM3 and .../ORB_SLAM3_ROS2 under the same parent directory.")
endif()

find_path(ORB_SLAM3_INCLUDE_DIR NAMES System.h
  PATHS ${ORB_SLAM3_ROOT_DIR}/include
  NO_DEFAULT_PATH)

find_library(ORB_SLAM3_LIBRARY NAMES ORB_SLAM3 libORB_SLAM3
  PATHS ${ORB_SLAM3_ROOT_DIR}/lib
  NO_DEFAULT_PATH)

find_path(DBoW2_INCLUDE_DIR NAMES Thirdparty/DBoW2/DBoW2/BowVector.h
  PATHS ${ORB_SLAM3_ROOT_DIR}
  NO_DEFAULT_PATH)

find_library(DBoW2_LIBRARY NAMES DBoW2 libDBoW2
  PATHS
    ${ORB_SLAM3_ROOT_DIR}/lib
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
  NO_DEFAULT_PATH)

find_library(g2o_LIBRARY NAMES g2o libg2o
  PATHS
    ${ORB_SLAM3_ROOT_DIR}/lib
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
  NO_DEFAULT_PATH)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ORB_SLAM3 DEFAULT_MSG
  ORB_SLAM3_LIBRARY ORB_SLAM3_INCLUDE_DIR DBoW2_INCLUDE_DIR DBoW2_LIBRARY g2o_LIBRARY)

mark_as_advanced(ORB_SLAM3_INCLUDE_DIR ORB_SLAM3_LIBRARY)

find_package(Eigen3 REQUIRED NO_MODULE)

set(ORB_SLAM3_LIBRARIES ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})
set(ORB_SLAM3_INCLUDE_DIRS
  ${ORB_SLAM3_INCLUDE_DIR}
  ${ORB_SLAM3_INCLUDE_DIR}/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Pangolin/include
  ${DBoW2_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS})
