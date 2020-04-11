#ifndef UCHILE_TF__POSE_TRANSFORMER__H
#define UCHILE_TF__POSE_TRANSFORMER__H

#include <string>
#include <memory>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <uchile_srvs/Transformer.h>

namespace uchile_tf
{
class PoseTransformer
{
public:
  PoseTransformer(ros::NodeHandle& node_handle, const ros::Duration& lookup_duration);
  virtual ~PoseTransformer();

  bool transformCallback(uchile_srvs::Transformer::Request& req, uchile_srvs::Transformer::Response& res);

private:
  ros::Duration lookup_duration_;
  ros::NodeHandle& node_handle_;
  ros::ServiceServer transform_server_;
  tf2_ros::Buffer buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

} /* namespace uchile_tf */

#endif  // UCHILE_TF__POSE_TRANSFORMER__H
