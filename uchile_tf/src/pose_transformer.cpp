#include <uchile_tf/pose_transformer.h>

namespace uchile_tf
{
PoseTransformer::PoseTransformer(std::string name) : name_(name)
{
  ros::NodeHandle priv("~");
  priv = ros::NodeHandle("~");

  transform_srv_ = priv.advertiseService("transform", &PoseTransformer::transform, this);
}

PoseTransformer::~PoseTransformer()
{
}

bool PoseTransformer::transform(uchile_srvs::Transformer::Request& req, uchile_srvs::Transformer::Response& res)
{
  tf::StampedTransform transform;
  int max_connectivity_exceptions = 5;
  int connectivity_exceptions = 0;

  while (ros::ok())
  {
    try
    {
      tf_listener_.waitForTransform(req.frame_out, req.pose_in.header.frame_id, req.pose_in.header.stamp,
                                    ros::Duration(2.0));
      tf_listener_.transformPose(req.frame_out, req.pose_in, res.pose_out);
      break;
    }
    catch (tf::ConnectivityException& e)
    {
      ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
      connectivity_exceptions++;

      if (connectivity_exceptions == max_connectivity_exceptions)
      {
        return false;
      }
      continue;
    }
    catch (tf::TransformException& e)
    {
      ROS_WARN_STREAM("Transform Exception: " << e.what());
      return false;
    }
  }
  return true;
}

}  // namespace uchile_tf

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_pose_transformer");

  uchile_tf::PoseTransformer* node = new uchile_tf::PoseTransformer(ros::this_node::getName());

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  ROS_INFO("Quitting ... \n");
  return 0;
}
