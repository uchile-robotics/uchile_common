#include <gtest/gtest.h>

#include <memory>
#include <uchile_tf/pose_transformer.h>
#include <tf2_ros/static_transform_broadcaster.h>

class PoseTransformer_Fixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ros::NodeHandle node_handle("~");
    unit = std::make_unique<uchile_tf::PoseTransformer>(node_handle, ros::Duration(0.1));
  }

  geometry_msgs::PoseStamped createPose(const std::string& frame_id)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id;
    pose.pose.orientation.w = 1.0;
    return pose;
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
    ROS_WARN_STREAM(tf);
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

  std::unique_ptr<uchile_tf::PoseTransformer> unit;
  uchile_srvs::Transformer::Request req{};
  uchile_srvs::Transformer::Response res{};
  tf2_ros::StaticTransformBroadcaster transform_broadcaster;
  float DELTA_F = 0.001;
};

TEST_F(PoseTransformer_Fixture, transformCallback_GivenSameFrames_ExpectSamePose)
{
  req.pose_in = createPose("frame_a");
  req.frame_out = "frame_a";

  EXPECT_TRUE(unit->transformCallback(req, res));
  EXPECT_EQ(req.pose_in, res.pose_out);
  EXPECT_EQ(req.pose_in.header.frame_id, req.frame_out);
}

TEST_F(PoseTransformer_Fixture, transformCallback_GivenNonExistentInputFrame_ExpectServiceError)
{
  req.pose_in = createPose("invalid_frame");
  req.frame_out = "frame_b";
  EXPECT_FALSE(unit->transformCallback(req, res));
}

TEST_F(PoseTransformer_Fixture, transformCallback_GivenNonExistentOutputFrame_ExpectServiceError)
{
  req.pose_in = createPose("frame_a");
  req.frame_out = "invalid_frame";
  EXPECT_FALSE(unit->transformCallback(req, res));
}

TEST_F(PoseTransformer_Fixture, transformCallback_GivenEmptyInputFrame_ExpectServiceError)
{
  req.pose_in = createPose("");
  req.frame_out = "frame_a";
  EXPECT_FALSE(unit->transformCallback(req, res));
}

TEST_F(PoseTransformer_Fixture, transformCallback_GivenEmptyOutputFrame_ExpectServiceError)
{
  req.pose_in = createPose("frame_a");
  req.frame_out = "";
  EXPECT_FALSE(unit->transformCallback(req, res));
}

TEST_F(PoseTransformer_Fixture, transformCallback_GivenDeltaX_ExpectTransformedPose)
{
  broadcastStaticTransform("parent", "child", 1.0F, 0.0F, 0.0F);
  req.pose_in = createPose("child");
  req.frame_out = "parent";

  EXPECT_TRUE(unit->transformCallback(req, res));
  EXPECT_POSITION(res.pose_out, 1.0, 0.0, 0.0);
  EXPECT_EQ(res.pose_out.header.frame_id, req.frame_out);
}

TEST_F(PoseTransformer_Fixture, transformCallback_GivenDeltaY_ExpectTransformedPose)
{
  broadcastStaticTransform("parent", "child", 0.0F, 1.0F, 0.0F);
  req.pose_in = createPose("child");
  req.frame_out = "parent";

  EXPECT_TRUE(unit->transformCallback(req, res));
  EXPECT_POSITION(res.pose_out, 0.0, 1.0, 0.0);
  EXPECT_EQ(res.pose_out.header.frame_id, req.frame_out);
}

TEST_F(PoseTransformer_Fixture, transformCallback_GivenDeltaTheta_ExpectTransformedPose)
{
  broadcastStaticTransform("parent", "child", 1.0F, 2.0F, M_PI / 2);
  req.pose_in = createPose("child");
  req.frame_out = "parent";

  EXPECT_TRUE(unit->transformCallback(req, res));
  EXPECT_EQ(res.pose_out.header.frame_id, req.frame_out);
  EXPECT_POSITION(res.pose_out, 1.0, 2.0, 0.0);
  EXPECT_ORIENTATION(res.pose_out, 0.0, 0.0, 0.7071, 0.7071);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_transformer_test");
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
