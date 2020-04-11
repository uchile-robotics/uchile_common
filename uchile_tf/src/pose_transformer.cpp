#include <uchile_tf/pose_transformer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace uchile_tf
{
PoseTransformer::PoseTransformer(ros::NodeHandle& node_handle, const ros::Duration& lookup_duration)
  : lookup_duration_(lookup_duration), node_handle_(node_handle)
{
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);
  transform_server_ = node_handle_.advertiseService("transform", &PoseTransformer::transformCallback, this);
}

PoseTransformer::~PoseTransformer()
{
}

bool PoseTransformer::transformCallback(uchile_srvs::Transformer::Request& req, uchile_srvs::Transformer::Response& res)
{
  geometry_msgs::TransformStamped transform;
  int max_connectivity_exceptions = 5;
  int connectivity_exceptions = 0;

  std::string frame_in = req.pose_in.header.frame_id;
  std::string frame_out = req.frame_out;
  ros::Time time_in = req.pose_in.header.stamp;

  if (frame_in == "" || frame_out == "")
  {
    return false;
  }

  while (ros::ok())
  {
    try
    {
      transform = buffer_.lookupTransform(frame_out, frame_in, time_in, lookup_duration_);
      tf2::doTransform(req.pose_in, res.pose_out, transform);
      break;
    }
    catch (tf2::ConnectivityException& e)
    {
      ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
      connectivity_exceptions++;

      if (connectivity_exceptions == max_connectivity_exceptions)
      {
        return false;
      }
      continue;
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN_STREAM("Transform Exception: " << e.what());
      return false;
    }
  }
  return true;
}

}  // namespace uchile_tf
