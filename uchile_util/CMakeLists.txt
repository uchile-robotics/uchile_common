cmake_minimum_required(VERSION 2.8.3)
project(uchile_util)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  uchile_msgs
  uchile_srvs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS 
    rospy
    std_msgs
    std_srvs
    uchile_msgs
    uchile_srvs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

# install header files under include/uchile_util
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
