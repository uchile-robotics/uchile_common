cmake_minimum_required(VERSION 2.8.3)
project(uchile_tf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  kdl_parser
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf_conversions
  uchile_msgs
  uchile_srvs
)
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

# Dynamic reconfigure files for Dynamic tf
generate_dynamic_reconfigure_options(
  cfg/tfParameters.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES
    dynamic_robot_state_publisher
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    kdl_parser
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf
    tf_conversions
    uchile_msgs
    uchile_srvs
  DEPENDS orocos_kdl
)

###########
## Build ##
###########

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(dynamic_robot_state_publisher
  src/dynamic_robot_state_publisher.cpp
  src/dynamic_tf.cpp
)
target_link_libraries(dynamic_robot_state_publisher ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# Pose transformer
add_executable(simple_pose_transformer src/TransformPoseSimple.cpp)
add_dependencies(simple_pose_transformer ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_pose_transformer ${catkin_LIBRARIES})

# Dynamic Robot State Publisher
add_executable(dyn_state_publisher src/dynamic_joint_state_listener.cpp)
# Generate cfg headers before
add_dependencies(dyn_state_publisher ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(dyn_state_publisher dynamic_robot_state_publisher ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES})
