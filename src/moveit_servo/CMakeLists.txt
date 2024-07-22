cmake_minimum_required(VERSION 3.1.3)
project(moveit_servo)
execute_process(
         COMMAND
             git rev-parse HEAD
         OUTPUT_VARIABLE
             GIT_HASH
             OUTPUT_STRIP_TRAILING_WHITESPACE)
add_definitions("-DGIT_HASH=\"${GIT_HASH}\"")
add_definitions("-DPROJECT_NAME=\"${PROJECT_NAME}\"")
add_definitions(-DPARALLEL_COLLISION_CHECK=1)
add_definitions(-DTHRESHOLD_PARALLEL_COLLISION_CHECK=1)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rosparam_shortcuts
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
  kdl_conversions
  dynamic_reconfigure
  kdl_parser
)
generate_dynamic_reconfigure_options(
        cfg/AngularSpeedScale.cfg
        cfg/LinearSpeedScale.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    control_msgs
    control_toolbox
    geometry_msgs
    moveit_msgs
    moveit_ros_planning_interface
    rosparam_shortcuts
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    trajectory_msgs
    dynamic_reconfigure
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

#########################################################
## Library to process realtime twist or joint commands ##
#########################################################

# This library provides an interface for sending realtime twist or joint commands to a robot
set(SERVO_CALC_STUFF
    src/servo_calcs/calc_single_iteration.cpp
    src/servo_calcs/joint_jog_control.cpp
    src/servo_calcs/cartesian_control.cpp
)

set(SERVO_LIB_NAME
  # These files are used to produce differential motion
  src/collision_check.cpp
  src/stop_distance_collision_check.cpp
  src/threshold_distance_check.cpp
  src/bullet_collision.cpp
  src/servo_calcs.cpp
  ${SERVO_CALC_STUFF}
  src/servo.cpp
  src/low_pass_filter.cpp

  src/command_publishers/float64_publisher.cpp
  src/command_publishers/trajectory_publisher.cpp
  src/command_publishers/pose_publisher.cpp
  src/command_publishers/command_publisher_interface.cpp

  src/kdl_based_extensions/command_validator.cpp
  src/extensions/value_converter.cpp

)

############################
## ROS message-based node ##
############################

add_executable(servo_server
  ${SERVO_LIB_NAME}
  src/servo_server.cpp
)
add_dependencies(servo_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(servo_server
  ${catkin_LIBRARIES}
)

add_executable(add_object
  ${SERVO_LIB_NAME}
  src/add_object.cpp
)
add_dependencies(add_object ${catkin_EXPORTED_TARGETS})
target_link_libraries(add_object
  ${catkin_LIBRARIES}
)


##################
## INSTALLATION ##
##################

install(
  TARGETS
    servo_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

