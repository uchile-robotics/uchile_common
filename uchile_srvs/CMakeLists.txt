cmake_minimum_required(VERSION 3.5)
project(uchile_srvs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(uchile_msgs REQUIRED)

# Archivos de servicio
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/BlobList.srv"
  "srv/DepthDetection.srv"
  "srv/DetectEmerg.srv"
  "srv/Detection.srv"
  "srv/DetectState.srv"
  "srv/DoorDetector.srv"
  "srv/FaceRecognition.srv"
  "srv/Float.srv"
  "srv/GenderDetection.srv"
  "srv/GetShelfPose.srv"
  "srv/Id.srv"
  "srv/ImageDetection.srv"
  "srv/ImageDetector.srv"
  "srv/ImageRecognition.srv"
  "srv/ImageReport.srv"
  "srv/ImageService.srv"
  "srv/IsOn.srv"
  "srv/LoadDictionary.srv"
  "srv/MaskInfo.srv"
  "srv/ObjectDetection.srv"
  "srv/ObjectRecognition.srv"
  "srv/OnOff.srv"
  "srv/PersonDetection.srv"
  "srv/PlaneHeight.srv"
  "srv/PlaySound.srv"
  "srv/PointCloud2.srv"
  "srv/PointCloud2Trasnform.srv"
  "srv/PoseStamped.srv"
  "srv/Recognition.srv"
  "srv/ReportGenerator.srv"
  "srv/RosbagRecord.srv"
  "srv/SearchPerson.srv"
  "srv/SiftClearDatabase.srv"
  "srv/SiftFileTrain.srv"
  "srv/SiftGuiTrain.srv"
  "srv/SiftOnlineTrain.srv"
  "srv/StringArray.srv"
  "srv/StringReturn.srv"
  "srv/String.srv"
  "srv/Synthesize.srv"
  "srv/TableDetector.srv"
  "srv/TimerString.srv"
  "srv/Transformer.srv"
  "srv/ValidPoint.srv"
  "action/Emotion.action"
  "action/LlamaSpeak.action"
  "action/SpeakEmotion.action"
  DEPENDENCIES std_msgs std_srvs geometry_msgs sensor_msgs uchile_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
