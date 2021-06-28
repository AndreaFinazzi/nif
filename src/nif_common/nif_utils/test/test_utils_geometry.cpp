//
// Created by usrg on 6/27/21.
//

#include "gtest/gtest.h"
#include "nif_utils/utils.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"


class UtilsGeometryTest : public ::testing::Test {
protected:
    void SetUp() override {
        pose_stamped_a.header.frame_id = "base_link";
        pose_stamped_b.header.frame_id = "base_link";

        pose_stamped_a.pose.position = point_a;
        pose_stamped_b.pose.position = point_b;
    }

    geometry_msgs::msg::Point point_a;
    geometry_msgs::msg::Quaternion quaternion_a;

    geometry_msgs::msg::Point point_b;
    geometry_msgs::msg::Quaternion quaternion_b;

    geometry_msgs::msg::PoseStamped pose_stamped_a, pose_stamped_b;


};

TEST_F(UtilsGeometryTest, EuclideanDistanceOne) {
    pose_stamped_a.pose.position.x = 0.0;
    pose_stamped_a.pose.position.y = 0.0;
    pose_stamped_a.pose.position.z = 0.0;

    pose_stamped_b.pose.position.x = 1.0;
    pose_stamped_b.pose.position.y = 1.0;
    pose_stamped_b.pose.position.z = 1.0;

    double pose_stamped_res = nif::common::utils::geometry::calEuclideanDistance(pose_stamped_a, pose_stamped_b);
    ASSERT_FLOAT_EQ(pose_stamped_res, 1.7320508076);
}