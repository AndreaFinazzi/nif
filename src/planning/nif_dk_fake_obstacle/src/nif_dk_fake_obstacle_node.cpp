/*
 * nif_dk_graph_planner_node.cpp
 *
 *  Created on: Oct 11, 2021
 *      Author: Daegyu Lee
 */
#include "nif_dk_fake_obstacle/nif_dk_fake_obstacle_node.h"
#include "nif_frame_id/frame_id.h"
#include <assert.h>
#include <string>

#define INF 9999999

using namespace nif::planning;
using namespace nif::common::frame_id::localization;

FakeObsNode::FakeObsNode(const std::string &node_name_)
    : Node(node_name_)
{
  this->declare_parameter<std::string>("fake_obs_osm_name", std::string(""));
  this->declare_parameter<double>("origin_lat", double(39.809786));
  this->declare_parameter<double>("origin_lon", double(-86.235148));

  this->m_OsmFileName = this->get_parameter("fake_obs_osm_name").as_string();
  this->m_originLat = this->get_parameter("origin_lat").as_double();
  this->m_originLon = this->get_parameter("origin_lon").as_double();

  // Set the ltp reference point
  nif::localization::utils::GeodeticConverter::GeoRef ref;
  ref.latitude = this->m_originLat;
  ref.longitude = this->m_originLon;
  ref.altitude = 0.;
  conv_.initializeReference(ref);

  using namespace std::chrono_literals; // NOLINT
  sub_timer_ = this->create_wall_timer(
      10ms, std::bind(&FakeObsNode::timer_callback, this));

  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/aw_localization/ekf/odom", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&FakeObsNode::OdometryCallback, this, std::placeholders::_1));
  pubFakeObsPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/fake_obs", nif::common::constants::QOS_SENSOR_DATA);
  pubFakeInflatedPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/inflated_points", nif::common::constants::QOS_SENSOR_DATA);
  pubFakeCenterPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cluster_center_points", nif::common::constants::QOS_SENSOR_DATA);
  pubGlobalFakeInflatedPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/glboal_inflated_points", nif::common::constants::QOS_SENSOR_DATA);

  OsmParcing();
}

FakeObsNode::~FakeObsNode() {}

void FakeObsNode::OsmParcing() {
  RCLCPP_INFO(this->get_logger(), "opening the osm map file..");
  // Open the file
  m_FullFilePath = m_OsmFileName.c_str();
  pugi::xml_document doc;
  pugi::xml_parse_result parcer = doc.load_file(m_FullFilePath.c_str());

  // file opening result
  if (!parcer) {
    RCLCPP_ERROR(this->get_logger(), "Invalid file path.. ");
    std::cout << "Parse error: " << parcer.description()
              << ", character pos= " << parcer.offset << std::endl;
    std::cout << "Tried to open .. \n" << m_FullFilePath.c_str() << std::endl;
  } else {
    std::cout << "Parse result: " << parcer.description()
              << ", character pos= " << parcer.offset << std::endl;
    std::cout << "Tried to open .. \n" << m_FullFilePath.c_str() << std::endl;
    RCLCPP_INFO(this->get_logger(), "Vaild file!");
  }


  // Node data
  for (pugi::xml_node node : doc.child("osm").children("node")) {
    // std::cout << node.attribute("id").value() << ", lat: " <<
    // node.attribute("lat").value() << ", lon: " <<
    // node.attribute("lon").value()<< std::endl;

    nif_dk_graph_planner_msgs::msg::Node nodeTmp;
    nodeTmp.id = node.attribute("id").as_int();
    nodeTmp.lat = node.attribute("lat").as_double();
    nodeTmp.lon = node.attribute("lon").as_double();

    nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
    currentGPS.latitude = (double)nodeTmp.lat;  
    currentGPS.longitude = (double)nodeTmp.lon; 
    // Currently ignore altitude for the most part and just track x/y
    currentGPS.altitude = 0.;
    nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
    conv_.geodetic2Ned(currentGPS, ltp_pt);

    nodeTmp.x = ltp_pt.x;
    nodeTmp.y = -ltp_pt.y;

    // Tags
    for (pugi::xml_node tag : node.children("tag")) {
      std::string start_layer = "start_layer";
      if (tag.attribute("k").as_string() == start_layer) {
        // std::cout << "yaw: " << tag.attribute("v").as_double() << std::endl;
        nodeTmp.start_layer = tag.attribute("v").as_int();
      }
      std::string end_layer = "end_layer";
      if (tag.attribute("k").as_string() == end_layer) {
        // std::cout << "yaw: " << tag.attribute("v").as_double() << std::endl;
        nodeTmp.end_layer = tag.attribute("v").as_int();
      }
      std::string start_node = "start_node";
      if (tag.attribute("k").as_string() == start_node) {
        // std::cout << "start_node: " << tag.attribute("v").as_double() <<
        // std::endl;
        nodeTmp.start_node = tag.attribute("v").as_int();
      }
    }
    m_OsmParcer.nodes.push_back(nodeTmp);
  }
  



  // // Way data
  nif_dk_graph_planner_msgs::msg::OsmParcer WayParcer;
  for (pugi::xml_node way : doc.child("osm").children("way")) {
    // Way id
    // std::cout << "---------------" << std::endl;
    // std::cout << "way id: " << way.attribute("id").value() << std::endl;
    nif_dk_graph_planner_msgs::msg::Way wayTmp;
    wayTmp.id = way.attribute("id").as_int();
    // attribute of way data : Node IDs
    for (pugi::xml_node way_data : way.children("nd")) {
      nif_dk_graph_planner_msgs::msg::Node nodeRef; // Node Ids in the each way
      nodeRef.id = way_data.attribute("ref").as_int();
      for (auto node : m_OsmParcer.nodes) {
        if (node.id == nodeRef.id) {
          nodeRef = node;
        }
      }
      wayTmp.nodes.push_back(nodeRef);
    }
    wayTmp.first_node_id = wayTmp.nodes[0].id;
    wayTmp.last_node_id = wayTmp.nodes[wayTmp.nodes.size() - 1].id;
    wayTmp.cost = INF;

    // Tags
    for (pugi::xml_node tag : way.children("tag")) {
      // start_layer
      std::string start_layer = "start_layer";
      // end_layer
      std::string end_layer = "end_layer";
      // start_node
      std::string start_node = "start_node";
      // end_node
      std::string end_node = "end_node";

      if (tag.attribute("k").as_string() == start_layer) {
        wayTmp.start_layer = tag.attribute("v").as_int();
      } else if (tag.attribute("k").as_string() == end_layer) {
        wayTmp.end_layer = tag.attribute("v").as_int();
      } else if (tag.attribute("k").as_string() == start_node) {
        wayTmp.start_node = tag.attribute("v").as_int();
      } else if (tag.attribute("k").as_string() == end_node) {
        wayTmp.end_node = tag.attribute("v").as_int();
      }
    }

    m_OsmParcer.ways.push_back(wayTmp);
  }

  m_FakeObsPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  for (auto node : m_OsmParcer.nodes) {
    pcl::PointXYZI current_point;
    current_point.x = node.x;
    current_point.y = node.y;
    current_point.intensity = node.start_layer;
    m_FakeObsPoints->points.push_back(current_point);
  }

  bParcingComplete = true;
}

void FakeObsNode::timer_callback() {
  if (bParcingComplete) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr fakeCenerPoints(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr fakeInflatedPoints(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr GlobalfakeInflatedPoints(
        new pcl::PointCloud<pcl::PointXYZI>);
    MessagePublisher();
    RCLCPP_INFO(this->get_logger(), "Publishing ------------");
    if (bOdometry){
      TransformPointsToBody(m_FakeObsPoints, fakeCenerPoints, m_veh_x, m_veh_y,
                            m_veh_yaw);
      createGaussianWorld(fakeCenerPoints, 3.0, 3.0, fakeInflatedPoints);

      sensor_msgs::msg::PointCloud2 FakeCenterCloudMsg;
      pcl::toROSMsg(*fakeCenerPoints, FakeCenterCloudMsg);
      FakeCenterCloudMsg.header.frame_id = 
          nif::common::frame_id::localization::BASE_LINK;
      FakeCenterCloudMsg.header.stamp = this->now();
      pubFakeCenterPoints->publish(FakeCenterCloudMsg);

      sensor_msgs::msg::PointCloud2 FakeInflatedCloudMsg;
      pcl::toROSMsg(*fakeInflatedPoints, FakeInflatedCloudMsg);
      FakeInflatedCloudMsg.header.frame_id =
          nif::common::frame_id::localization::BASE_LINK;
      FakeInflatedCloudMsg.header.stamp = this->now();
      pubFakeInflatedPoints->publish(FakeInflatedCloudMsg);
                           
      sensor_msgs::msg::PointCloud2 GlobalFakeInflatedCloudMsg;
      TransformPointsToGlobal(fakeInflatedPoints, GlobalfakeInflatedPoints,
                              m_veh_x, m_veh_y, m_veh_yaw);
      pcl::toROSMsg(*GlobalfakeInflatedPoints, GlobalFakeInflatedCloudMsg);
      GlobalFakeInflatedCloudMsg.header.frame_id =
          nif::common::frame_id::localization::ODOM;
      GlobalFakeInflatedCloudMsg.header.stamp = this->now();
      pubGlobalFakeInflatedPoints->publish(GlobalFakeInflatedCloudMsg);

      RCLCPP_INFO(this->get_logger(), "Publishing Fake inflated obs");

    }

    MessagePublisher();
    RCLCPP_INFO(this->get_logger(), "Publishing Fake Obs");

  }
}

void FakeObsNode::OdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  m_veh_x = msg->pose.pose.position.x;
  m_veh_y = msg->pose.pose.position.y;

  tf2::Quaternion tf_quat;
  tf2::convert(msg->pose.pose.orientation, tf_quat);
  tf2::Matrix3x3 mat(tf_quat);
  mat.getRPY(m_veh_roll, m_veh_pitch, m_veh_yaw);
  bOdometry = true;

}

void FakeObsNode::MessagePublisher() {
  // origin node publisher  
  sensor_msgs::msg::PointCloud2 FullNodeCloudMsg;
  pcl::toROSMsg(*m_FakeObsPoints, FullNodeCloudMsg);
  FullNodeCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
  FullNodeCloudMsg.header.stamp = this->now();
  pubFakeObsPoints->publish(FullNodeCloudMsg);
}

void FakeObsNode::TransformPointsToBody(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIn,
    pcl::PointCloud<pcl::PointXYZI>::Ptr CloudOut, const double &veh_x_,
    const double &veh_y_, const double &veh_yaw_) {

  for (auto point : CloudIn->points) {
    pcl::PointXYZI pointOnBody;
    pointOnBody.x =
        (point.x - veh_x_) * cos(veh_yaw_) + (point.y - veh_y_) * sin(veh_yaw_);
    pointOnBody.y = -(point.x - veh_x_) * sin(veh_yaw_) +
                    (point.y - veh_y_) * cos(veh_yaw_);
    pointOnBody.z = point.z;
    CloudOut->points.push_back(pointOnBody);
  }
}

// Calculate gaussian interpolation.
void FakeObsNode::createGaussianWorld(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &points_in, double inflation_x,
    double inflation_y, pcl::PointCloud<pcl::PointXYZI>::Ptr &points_out) {

  struct Gaussian {
    double x0, y0;
    double varX, varY;
    double s;
  };

  AnalyticalFunctions func;
  std::vector<std::pair<double, double>> vars;
  std::vector<std::pair<double, double>> means;
  std::vector<double> scales;
  std::vector<Gaussian> g;

  for (auto point : points_in->points) {
    Gaussian gaussian_tmp;
    gaussian_tmp.x0 = point.x;
    gaussian_tmp.y0 = point.y;
    gaussian_tmp.varX = inflation_x;
    gaussian_tmp.varY = inflation_y;
    gaussian_tmp.s = 1 / inflation_x;
    g.push_back(gaussian_tmp);
  }

  func.f_ = [g](double x, double y) {
    double value = 0.0;
    for (int i = 0; i < g.size(); ++i) {
      const double x0 = g.at(i).x0;
      const double y0 = g.at(i).y0;
      const double varX = g.at(i).varX;
      const double varY = g.at(i).varY;
      const double s = g.at(i).s;
      value += s * std::exp(-(x - x0) * (x - x0) / (2.0 * varX) -
                            (y - y0) * (y - y0) / (2.0 * varY));
    }
    return value;
  };

  for (auto point : points_in->points) {
    for (double i = -inflation_x; i < inflation_x; i = i + 0.5) {
      for (double j = -inflation_y; j < inflation_y; j = j + 0.5) {
        pcl::PointXYZI point_buf;
        point_buf.x = point.x + i;
        point_buf.y = point.y + j;
        point_buf.intensity = func.f_(point_buf.x, point_buf.y);
        if (point_buf.intensity < fabs(1.0))
          points_out->points.push_back(point_buf);
      }
    }
  }
  // return output;
}

void FakeObsNode::TransformPointsToGlobal(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudIn,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudOut, const double &veh_x_,
    const double &veh_y_, const double &veh_yaw_) {

  for (auto point : CloudIn->points) {
    pcl::PointXYZI pointOnGlobal;
    pointOnGlobal.x =
        point.x * cos(veh_yaw_) - point.y * sin(veh_yaw_) + veh_x_;
    pointOnGlobal.y =
        point.x * sin(veh_yaw_) + point.y * cos(veh_yaw_) + veh_y_;
    pointOnGlobal.z = point.z;
    pointOnGlobal.intensity = point.intensity;
    CloudOut->points.push_back(pointOnGlobal);
  }
}