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
      200ms, std::bind(&FakeObsNode::timer_callback, this));
          
  pubFakeObsPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/fake_obs", nif::common::constants::QOS_SENSOR_DATA);
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
    MessagePublisher();
    RCLCPP_INFO(this->get_logger(), "Publishing Fake Obs");
  }
}

void FakeObsNode::MessagePublisher() {
  // origin node publisher  
  sensor_msgs::msg::PointCloud2 FullNodeCloudMsg;
  pcl::toROSMsg(*m_FakeObsPoints, FullNodeCloudMsg);
  FullNodeCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
  FullNodeCloudMsg.header.stamp = this->now();
  pubFakeObsPoints->publish(FullNodeCloudMsg);
}
