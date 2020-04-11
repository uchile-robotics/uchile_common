#include <ros/ros.h>
#include <uchile_tf/pose_transformer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_pose_transformer");

  ros::NodeHandle node_handle("~");
  ros::Duration lookup_duration(2.0);
  uchile_tf::PoseTransformer node(node_handle, lookup_duration);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  ROS_INFO("Quitting ... \n");
  return 0;
}
