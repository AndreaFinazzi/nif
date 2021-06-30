//
// Created by usrg on 6/27/21.
//

#include "gtest/gtest.h"
#include "nif_utils/utils.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

const double DISTANCE_ABS_ERROR = 0.0001;
const double VELOCITY_ABS_ERROR = 0.0005;

class UtilsGeometryEuclideanTest : public ::testing::Test {
protected:
    /**
     * Called by GTest at the beginning of the test
     */
    void SetUp() override {
        pose_stamped_a.header.frame_id = "base_link";
        pose_stamped_b.header.frame_id = "base_link";

        point_a.x = 0.0;
        point_a.y = 0.0;
        point_a.z = 0.0;

        point_b.x = 1.0;
        point_b.y = 1.0;
        point_b.z = 1.0;

        pose_a.position = point_a;
        pose_b.position = point_b;

        pose_stamped_a.pose = pose_a;
        pose_stamped_b.pose = pose_b;

    }

    /**
     * Called by GTest at the end of the test
     */
    void TearDown() override {

    }

    geometry_msgs::msg::Point point_a, point_b;
    geometry_msgs::msg::Pose pose_a, pose_b;
    geometry_msgs::msg::PoseStamped pose_stamped_a, pose_stamped_b;

};

TEST_F(UtilsGeometryEuclideanTest, EuclideanDistancePoseStamped) {

    double pose_stamped_res = nif::common::utils::geometry::calEuclideanDistance(pose_stamped_a, pose_stamped_b);
    ASSERT_NEAR(pose_stamped_res, 1.7320508076, DISTANCE_ABS_ERROR);
}

TEST_F(UtilsGeometryEuclideanTest, EuclideanDistancePose) {

    double pose_stamped_res = nif::common::utils::geometry::calEuclideanDistance(pose_a, pose_b);
    ASSERT_NEAR(pose_stamped_res, 1.7320508076, DISTANCE_ABS_ERROR);
}

TEST_F(UtilsGeometryEuclideanTest, EuclideanDistancePoint) {

    double pose_stamped_res = nif::common::utils::geometry::calEuclideanDistance(point_a, point_b);
    ASSERT_NEAR(pose_stamped_res, 1.7320508076, DISTANCE_ABS_ERROR);
}

TEST(UtilsGeometryVelocityTest, VelocityTest) {
    {
        double velocity_res = nif::common::utils::geometry::kph2mph(100.0);
        ASSERT_NEAR(velocity_res, 62.137119, VELOCITY_ABS_ERROR);
    }
    {
        double velocity_res = nif::common::utils::geometry::mph2kph(62.137119);
        ASSERT_NEAR(velocity_res, 100.0, VELOCITY_ABS_ERROR);
    }
}