#include <gtest/gtest.h>

#include <memory>

#include <ros/ros.h>
#include <ros/service_client.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <uchile_srvs/Transformer.h>

class PoseTransformerNode_Fixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_handle = std::make_unique<ros::NodeHandle>("~");
    transform_client = node_handle->serviceClient<uchile_srvs::Transformer>("/pose_transformer_node/transform");
  }

  geometry_msgs::PoseStamped createPose(const std::string& frame_id)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  void waitForExistence()
  {
    bool exists(transform_client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);
  }

  void broadcastStaticTransform(const std::string& parent, const std::string& child, const float dx, const float dy,
                                const float dtheta)
  {
    geometry_msgs::TransformStamped tf{};
    tf.header.frame_id = parent;
    tf.child_frame_id = child;

    tf.transform.translation.x = dx;
    tf.transform.translation.y = dy;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, dtheta);
    tf.transform.rotation.x = quat.x();
    tf.transform.rotation.y = quat.y();
    tf.transform.rotation.z = quat.z();
    tf.transform.rotation.w = quat.w();
    tf.header.stamp = ros::Time::now();
    transform_broadcaster.sendTransform(tf);
  }

  void EXPECT_POSITION(const geometry_msgs::PoseStamped& pose_stamped, const float x, const float y, const float z)
  {
    auto position = &pose_stamped.pose.position;
    EXPECT_NEAR(position->x, x, DELTA_F);
    EXPECT_NEAR(position->y, y, DELTA_F);
    EXPECT_NEAR(position->z, z, DELTA_F);
  }

  void EXPECT_ORIENTATION(const geometry_msgs::PoseStamped& pose_stamped, const float qx, const float qy,
                          const float qz, const float qw)
  {
    auto orientation = &pose_stamped.pose.orientation;
    EXPECT_NEAR(orientation->x, qx, DELTA_F);
    EXPECT_NEAR(orientation->y, qy, DELTA_F);
    EXPECT_NEAR(orientation->z, qz, DELTA_F);
    EXPECT_NEAR(orientation->w, qw, DELTA_F);
  }

  std::unique_ptr<ros::NodeHandle> node_handle;
  ros::ServiceClient transform_client;
  uchile_srvs::Transformer::Request req{};
  uchile_srvs::Transformer::Response res{};
  tf2_ros::StaticTransformBroadcaster transform_broadcaster;
  float DELTA_F = 0.001;
};

TEST_F(PoseTransformerNode_Fixture, clientCall_GivenSameFrames_ExpectSamePose)
{
  req.pose_in = createPose("frame_a");
  req.frame_out = "frame_a";

  waitForExistence();
  EXPECT_TRUE(transform_client.call(req, res));
  EXPECT_EQ(req.pose_in, res.pose_out);
  EXPECT_EQ(req.pose_in.header.frame_id, req.frame_out);
}

TEST_F(PoseTransformerNode_Fixture, clientCall_GivenNonExistentInputFrame_ExpectServiceError)
{
  broadcastStaticTransform("frame_a", "frame_b", 0.0F, 0.0F, 0.0F);
  req.pose_in = createPose("invalid_frame");
  req.frame_out = "frame_b";

  waitForExistence();
  EXPECT_FALSE(transform_client.call(req, res));
}

TEST_F(PoseTransformerNode_Fixture, clientCall_GivenNonExistentOutputFrame_ExpectServiceError)
{
  broadcastStaticTransform("frame_a", "frame_b", 0.0F, 0.0F, 0.0F);
  req.pose_in = createPose("frame_a");
  req.frame_out = "invalid_frame";

  waitForExistence();
  EXPECT_FALSE(transform_client.call(req, res));
}

TEST_F(PoseTransformerNode_Fixture, clientCall_GivenEmptyInputFrame_ExpectServiceError)
{
  broadcastStaticTransform("frame_a", "frame_b", 0.0F, 0.0F, 0.0F);
  req.pose_in = createPose("");
  req.frame_out = "frame_a";

  waitForExistence();
  EXPECT_FALSE(transform_client.call(req, res));
}

TEST_F(PoseTransformerNode_Fixture, clientCall_GivenEmptyOutputFrame_ExpectServiceError)
{
  broadcastStaticTransform("frame_a", "frame_b", 0.0F, 0.0F, 0.0F);
  req.pose_in = createPose("frame_a");
  req.frame_out = "";

  waitForExistence();
  EXPECT_FALSE(transform_client.call(req, res));
}

TEST_F(PoseTransformerNode_Fixture, clientCall_GivenTransform_ExpectTransformedPose)
{
  broadcastStaticTransform("parent", "child", 1.0F, 2.0F, M_PI / 2);
  req.pose_in = createPose("child");
  req.frame_out = "parent";

  waitForExistence();
  EXPECT_TRUE(transform_client.call(req, res));
  EXPECT_EQ(res.pose_out.header.frame_id, req.frame_out);
  EXPECT_POSITION(res.pose_out, 1.0, 2.0, 0.0);
  EXPECT_ORIENTATION(res.pose_out, 0.0, 0.0, 0.7071, 0.7071);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_transformer_node_test");
  srand((int)time(0));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
