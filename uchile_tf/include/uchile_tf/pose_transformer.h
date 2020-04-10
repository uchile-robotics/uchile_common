#ifndef UCHILE_TF__POSE_TRANSFORMER__H
#define UCHILE_TF__POSE_TRANSFORMER__H

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_listener.h>
#include <uchile_srvs/Transformer.h>

namespace uchile_tf
{
class PoseTransformer
{
public:
  PoseTransformer(std::string name);
  ~PoseTransformer();

private:
  ros::ServiceServer transform_srv_;
  tf::TransformListener tf_listener_;
  std::string name_;

private:
  bool transform(uchile_srvs::Transformer::Request& req, uchile_srvs::Transformer::Response& res);
};

} /* namespace uchile_tf */

#endif  // UCHILE_TF__POSE_TRANSFORMER__H
