cmake_minimum_required(VERSION 3.5)
project(uchile_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Mensajes
rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/AnimalCard.msg"
  "msg/BlobList.msg"
  "msg/Command.msg"
  "msg/CylindricalObject.msg"
  "msg/DataFollow.msg"
  "msg/Detections.msg"
  "msg/EmotionDetect.msg"
  "msg/ImageArray.msg"
  "msg/KinectTrackerData.msg"
  "msg/LedCommand2.msg"
  "msg/LedCommand.msg"
  "msg/ObjectDetection.msg"
  "msg/OclusionData.msg"
  "msg/PathBlockingDetections.msg"
  "msg/PlaneDetection.msg"
  "msg/PoseDetections.msg"
  "msg/Rect.msg"
  "msg/ROIArray.msg"
  "msg/SemanticPolygons.msg"
  "msg/SiftDetectionDepth.msg"
  "msg/SiftDetectionFloor.msg"
  "msg/SiftDetection.msg"
  "msg/SkeletonWeb.msg"
  "msg/StateInfo.msg"
  "msg/StringArray.msg"
  "msg/WaveData.msg"

  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
