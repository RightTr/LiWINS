cmake_minimum_required(VERSION 3.16)
project(liwins)

SET(CMAKE_BUILD_TYPE "Release")

set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(GTSAM REQUIRED QUIET)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  liwins
  liwins/utils
  liwins/core)

set(LIWINS_CORE_SOURCES
  liwins/core/laserMapping.cpp
  liwins/core/liwinsCalib.cpp
  liwins/sensors/preprocess.cpp
  liwins/sensors/IMU_Processing.cpp
  liwins/sensors/wheel_Processing.cpp
  liwins/map/ikd-Tree/ikd_Tree.cpp
  liwins/map/octvox/octvox.cpp
  liwins/filter/filter.cpp
  liwins/lio_sam/utility.cpp
  liwins/lio_sam/mapOptimization.cpp
  liwins/ros_interface/ros_interface.cpp
  liwins/calib/lidar_imu_wheel_init.cpp)

set(LIWINS_APP
  liwins/app/liwins.cpp)

set(LIWINS_CALIB_APP
  liwins/app/calib.cpp)


find_package(catkin QUIET)
if(catkin_FOUND)
  message(STATUS "ROS1 detected")
    set(USE_ROS1 ON)
endif()

find_package(ament_cmake QUIET)
  if(ament_cmake_FOUND)
  message(STATUS "ROS2 detected")
  set(USE_ROS2 ON)
endif()

if(USE_ROS1)
  add_definitions(-DUSE_ROS1)
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    sensor_msgs
    roscpp
    rospy
    std_msgs
    pcl_ros
    tf
    livox_ros_driver2
    message_generation
    eigen_conversions
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

  add_message_files(
    FILES
    Pose6D.msg
    Wheel.msg
  )

  generate_messages(
  DEPENDENCIES
  geometry_msgs
  )

  catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
    DEPENDS EIGEN3 PCL GTSAM
    INCLUDE_DIRS
  )

  add_executable(liwins_mapping
    ${LIWINS_APP}
    ${LIWINS_CORE_SOURCES})
  target_link_libraries(liwins_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

  add_executable(liwins_calib
    ${LIWINS_CALIB_APP}
    ${LIWINS_CORE_SOURCES})
  target_link_libraries(liwins_calib ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)
endif()

if(USE_ROS2)
  add_definitions(-DUSE_ROS2)
  find_package(rosidl_default_generators REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(tf2_eigen REQUIRED)
  find_package(livox_ros_driver2)
  find_package(visualization_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Pose6D.msg"
    "msg/Wheel.msg"
    DEPENDENCIES
      geometry_msgs
  )
  
  add_executable(liwins_mapping
    ${LIWINS_APP}
    ${LIWINS_CORE_SOURCES})
  add_executable(liwins_calib
    ${LIWINS_CALIB_APP}
    ${LIWINS_CORE_SOURCES})
  ament_target_dependencies(liwins_mapping
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    pcl_conversions
    PCL
    tf2_eigen
    livox_ros_driver2
    visualization_msgs
  )
  ament_target_dependencies(liwins_calib
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    pcl_conversions
    PCL
    tf2_eigen
    livox_ros_driver2
    visualization_msgs
  )
  target_include_directories(liwins_mapping PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )
  target_include_directories(liwins_calib PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(liwins_mapping ${PCL_LIBRARIES} gtsam ${cpp_typesupport_target})
  target_link_libraries(liwins_calib ${PCL_LIBRARIES} gtsam ${cpp_typesupport_target})
  install(
  DIRECTORY
    launch_ROS2
    config
    DESTINATION share/${PROJECT_NAME}
  )
  install(
    FILES
    rviz_cfg/loam_ros2.rviz
    rviz_cfg/sam_ros2.rviz
    rviz_cfg/calib_ros2.rviz
    DESTINATION share/${PROJECT_NAME}/rviz_cfg
  )
  install(TARGETS liwins_mapping liwins_calib
    DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY liwins/ DESTINATION include)

  ament_export_dependencies(rosidl_default_runtime)
  ament_package() # Don't forget this
endif()
