cmake_minimum_required(VERSION 2.8.3)
project(uchile_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  std_srvs
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
    baseArmState.msg
    baseEfectorPosition.msg
    BlobList.msg
    CartesianInfo.msg
    Command.msg
    CylindricalObject.msg
    data_follow.msg
    detections.msg
    EmotionDetect.msg
    FaceEmotion.msg
    FKinfo.msg
    grip.msg
    KinectTrackerData.msg
    LedCommand.msg 
    LedCommand2.msg 
    Marker.msg 
    MarkerArray.msg 
    Num.msg 
    ObjectDetection.msg
    OclusionData.msg
    PathBlockingDetections.msg
    PlaneDetection.msg
    PoseDetections.msg
    Rect.msg
    ROIArray.msg
    SemanticObject.msg
    ServoCommand.msg 
    ServoState.msg 
    SiftDetection.msg
    SiftDetectionDepth.msg
    SiftDetectionFloor.msg
    StateInfo.msg
    WaveData.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    sensor_msgs
    std_msgs
    std_srvs
)
