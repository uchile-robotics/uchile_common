cmake_minimum_required(VERSION 2.8.3)
project(uchile_srvs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  std_srvs
  uchile_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################



# Generate services in the 'srv' folder
add_service_files(
  FILES
  BlobList.srv
  Bool.srv
  DetectEmerg.srv
  Detection.srv
  DetectState.srv
  DoorDetector.srv
  FaceRecognition.srv
  Float.srv
  GenderDetection.srv
  GetShelfPose.srv
  ID.srv
  ImageDetection.srv
  ImageDetector.srv
  ImageRecognition.srv
  ImageReport.srv
  ImageService.srv
  IsOn.srv
  load_dictionary_service.srv
  MaskInfo.srv
  NavGoal.srv
  ObjectDetection.srv
  objrecService.srv
  Onoff.srv
  PlaneHeight.srv
  play_sound.srv
  PointCloud2.srv
  PointCloud2Trasnform.srv
  PoseStamped.srv
  Recognition.srv
  ReportGenerator.srv
  RosbagRecord.srv
  SearchPerson.srv
  SemMap.srv
  siftClearDatabase.srv
  siftFileTrain.srv
  siftGuiTrain.srv
  siftOnlineTrain.srv
  String.srv
  StringArray.srv
  stringReturn.srv
  stringService.srv
  synthesize.srv
  TableDetector.srv
  TimerString.srv
  Transformer.srv
  ValidPoint.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
    uchile_msgs
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
    uchile_msgs
)