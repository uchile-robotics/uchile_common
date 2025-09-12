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
  "srv/Bool.srv"
  "srv/DetectEmerg.srv"
  "srv/Detection.srv"
  "srv/DetectState.srv"
  "srv/DepthDetection.srv"
  "srv/DoorDetector.srv"
  "srv/FaceRecognition.srv"
  "srv/Float.srv"
  "srv/GenderDetection.srv"
  "srv/GetShelfPose.srv"
  "srv/ID.srv"
  "srv/ImageDetection.srv"
  "srv/ImageDetector.srv"
  "srv/ImageRecognition.srv"
  "srv/ImageReport.srv"
  "srv/ImageService.srv"
  "srv/IsOn.srv"
  "srv/load_dictionary_service.srv"
  "srv/MaskInfo.srv"
  "srv/NavGoal.srv"
  "srv/ObjectDetection.srv"
  "srv/objrecService.srv"
  "srv/Onoff.srv"
  "srv/PlaneHeight.srv"
  "srv/play_sound.srv"
  "srv/PointCloud2.srv"
  "srv/PointCloud2Trasnform.srv"
  "srv/PoseStamped.srv"
  "srv/Recognition.srv"
  "srv/ReportGenerator.srv"
  "srv/RosbagRecord.srv"
  "srv/SearchPerson.srv"
  "srv/SemMap.srv"
  "srv/siftClearDatabase.srv"
  "srv/siftFileTrain.srv"
  "srv/siftGuiTrain.srv"
  "srv/siftOnlineTrain.srv"
  "srv/String.srv"
  "srv/StringArray.srv"
  "srv/stringReturn.srv"
  "srv/stringService.srv"
  "srv/synthesize.srv"
  "srv/TableDetector.srv"
  "srv/TimerString.srv"
  "srv/Transformer.srv"
  "srv/ValidPoint.srv"
  "srv/PersonDetection.srv"
  DEPENDENCIES std_msgs std_srvs geometry_msgs sensor_msgs uchile_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
