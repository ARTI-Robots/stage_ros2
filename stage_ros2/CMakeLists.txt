cmake_minimum_required(VERSION 3.5)
project(stage_ros2)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if ($ENV{ROS_DISTRO} STREQUAL "galactic" OR $ENV{ROS_DISTRO} STREQUAL "foxy")
  add_compile_options(-DUSE_LEGACY_MSGS_INCLUDE)
endif ()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mrpt_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stage_ros2_itfs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# We must use lowercase Stage as the argument for the find_package below.
# Both works but they result in different variable being set `Stage_DIR` vs `stage_DIR`,
# and variable names are case-sensitive. Stage's stage-config.cmake file incorrectly uses ${stage_DIR}.
find_package(stage REQUIRED)
find_package(backward_ros REQUIRED)

# build
add_executable(${PROJECT_NAME}
    src/camera_wrapper.cc include/stage_ros2/camera_wrapper.hpp
    src/fiducial_wrapper.cc include/stage_ros2/fiducial_wrapper.hpp
    src/model_server.cc include/stage_ros2/model_server.hpp
    src/model_wrapper.cc include/stage_ros2/model_wrapper.hpp
    src/position_wrapper.cc include/stage_ros2/position_wrapper.hpp
    src/ranger_wrapper.cc include/stage_ros2/ranger_wrapper.hpp
    include/stage_ros2/stage_forward_declarations.hpp
    src/stage_ros2.cc
    src/utils.cc include/stage_ros2/utils.hpp)

# Add Stage dependency (the STAGE_* variables are defined by stage-config.cmake),
# which was found by `find_package(stage REQUIRED)`.
target_include_directories(${PROJECT_NAME} PRIVATE
    ${Eigen3_INCLUDE_DIRS}
    ${STAGE_INCLUDE_DIRS}
    include)
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${Eigen3_LIBRARIES}
    ${STAGE_LIBRARIES})
# Add dependencies on standard ROS 2 packages using ament.
ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
    rclcpp
    rclcpp_action
    rcutils
    cv_bridge
    image_transport
    geometry_msgs
    mrpt_msgs
    nav_msgs
    sensor_msgs
    stage_ros2_itfs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs)

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

ament_package()
