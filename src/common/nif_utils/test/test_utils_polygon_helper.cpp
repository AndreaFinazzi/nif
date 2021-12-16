//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/27/21.
//

#include "nif_utils/polygon_helper.h"
#include "gtest/gtest.h"

const double DISTANCE_ABS_ERROR = 0.0001;
const double VELOCITY_ABS_ERROR = 0.0005;

using nif::utils::geometry::Point2D;


// // Driver program to test above functions
// int main()
// {
// 	Point2D polygon1[] = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
// 	int n = sizeof(polygon1)/sizeof(polygon1[0]);
// 	Point2D p = {20, 20};
// 	isInside(polygon1, n, p)? cout << "Yes \n": cout << "No \n";

// 	p = {5, 5};
// 	isInside(polygon1, n, p)? cout << "Yes \n": cout << "No \n";

// 	Point2D polygon2[] = {{0, 0}, {5, 5}, {5, 0}};
// 	p = {3, 3};
// 	n = sizeof(polygon2)/sizeof(polygon2[0]);
// 	isInside(polygon2, n, p)? cout << "Yes \n": cout << "No \n";

// 	p = {5, 1};
// 	isInside(polygon2, n, p)? cout << "Yes \n": cout << "No \n";

// 	p = {8, 1};
// 	isInside(polygon2, n, p)? cout << "Yes \n": cout << "No \n";

// 	Point2D polygon3[] = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
// 	p = {-1,10};
// 	n = sizeof(polygon3)/sizeof(polygon3[0]);
// 	isInside(polygon3, n, p)? cout << "Yes \n": cout << "No \n";

// 	return 0;
// }

class UtilsPolygonHelperTest : public ::testing::Test {
protected:
  /**
   * Called by GTest at the beginning of the test
   */
  void SetUp() override {

  }

  /**
   * Called by GTest at the end of the test
   */
  void TearDown() override {}

  Point2D point_a = {-1.75, 3.1};   // Out of both; Aligned with edge;
  Point2D point_b = {4.1, 3.1};     // In of both; On vertex;
  Point2D point_c = {3.2, 1.5};     // In of both;
  Point2D point_d = {3.4025, 0.0};  // In of both; On edge;
  Point2D point_e = {3.2, -1.0};    // Out of both; No intersec;
  Point2D point_f = {0.0, -0.9};    // Out of both; Intersec vertex 0;
  Point2D point_g = {3.2, 6.0};     // Out of both; Intersec vertex b2;
  Point2D point_h = {3.2, 3.1};     // In of both; Intersec verteces and edge;

  std::vector<Point2D> polygon_a =
    {
      {3.2, -0.9},
      {-1.5, -2.3},
      {-2.0, 0.0},
      {-1.5, 3.1},
      {4.1, 3.1}
    };

  std::vector<Point2D> polygon_b = 
    {
      {3.2, -0.9},
      {0.0, 4.3},
      {6.0, 6.0},
      {4.1, 3.1}
    };

};

TEST_F(UtilsPolygonHelperTest, PolygonPointIn) {
  
  ASSERT_TRUE(nif::utils::geometry::poly::isInside(
    polygon_a, polygon_a.size(), point_b
  ));
  
  ASSERT_TRUE(nif::utils::geometry::poly::isInside(
    polygon_b, polygon_b.size(), point_b
  ));

  ASSERT_TRUE(nif::utils::geometry::poly::isInside(
    polygon_a, polygon_a.size(), point_c
  ));
  ASSERT_TRUE(nif::utils::geometry::poly::isInside(
    polygon_b, polygon_b.size(), point_c
  ));

  ASSERT_TRUE(nif::utils::geometry::poly::isInside(
    polygon_a, polygon_a.size(), point_d
  ));
  
  ASSERT_TRUE(nif::utils::geometry::poly::isInside(
    polygon_b, polygon_b.size(), point_d
  ));

  ASSERT_TRUE(nif::utils::geometry::poly::isInside(
    polygon_a, polygon_b.size(), point_f
  ));
  
  ASSERT_TRUE(nif::utils::geometry::poly::isInside(
    polygon_a, polygon_a.size(), point_h
  ));
  
  ASSERT_TRUE(nif::utils::geometry::poly::isInside(
    polygon_b, polygon_b.size(), point_h
  ));
}


TEST_F(UtilsPolygonHelperTest, PolygonPointOut) {

  ASSERT_TRUE(! nif::utils::geometry::poly::isInside(
    polygon_a, polygon_a.size(), point_a
  ));

  ASSERT_TRUE(! nif::utils::geometry::poly::isInside(
    polygon_b, polygon_b.size(), point_a
  ));

  ASSERT_TRUE(! nif::utils::geometry::poly::isInside(
    polygon_a, polygon_a.size(), point_e
  ));

  ASSERT_TRUE(! nif::utils::geometry::poly::isInside(
    polygon_b, polygon_b.size(), point_e
  ));

  ASSERT_TRUE(! nif::utils::geometry::poly::isInside(
    polygon_b, polygon_b.size(), point_f
  ));
  
  ASSERT_TRUE(! nif::utils::geometry::poly::isInside(
    polygon_a, polygon_a.size(), point_g
  ));

  ASSERT_TRUE(! nif::utils::geometry::poly::isInside(
    polygon_b, polygon_b.size(), point_g
  ));
}
