#include "eufs_sim2/plugin/gnss.hpp"

#include <geodesy/utm.h>    // NOLINT
#include <geodesy/wgs84.h>  // NOLINT

#include <functional>

#include "eufs_sim2/simulation.hpp"
#include "eufs_sim2/time/time.hpp"

using eufs::sim2::sensors::GNSSCoordinatesMember::_lat;
using eufs::sim2::sensors::GNSSCoordinatesMember::_lng;
using VehicleState = eufs::sim2::core::VehicleState;
using GeoPoint = geographic_msgs::msg::GeoPoint;

namespace eufs::sim2::plugin {

void GNSSPlugin::ThrottedPostUpdate(const SimulationBase &sim) {
  time::Time time = sim.GetCore().GetTime();
  VehicleState::Vector sensor_data;
  sim.GetCore().GetState(sensor_data);
  publisher_->publish(GNSSPlugin::ConvertToMsg(GetGNSS(sensor_data), time));
}

GeoPoint GNSSPlugin::GetGNSS(const VehicleState::Vector &vehicle_state) {
  sensors::WGSCoordinates wgsCoordinates;
  geographic_msgs::msg::GeoPoint geo_point;
  double easting = 488209.34 + vehicle_state(type::VehicleStateMember::_x);
  double northing = 6199824.55 + vehicle_state(type::VehicleStateMember::_y);
  geodesy::UTMPoint utm_point(easting, northing, 30, 'u');

  // Convert UTM to WGS84
  auto wgs = toMsg(utm_point);

  return wgs;
}

void GNSSPlugin::SetupROS(rclcpp::Node::SharedPtr node) {
  node_ = node->create_sub_node(subnode_name_);
  publisher_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/plugin/" + subnode_name_ + "/fix", 10);
  frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 200);
  RCLCPP_INFO(node_->get_logger(), "GNSS plugin initialised!");
}

sensor_msgs::msg::NavSatFix GNSSPlugin::ConvertToMsg(GeoPoint gp, time::Time t) {
  sensor_msgs::msg::NavSatFix msg;
  msg.header.frame_id = "gnss";
  msg.header.stamp = time::TimeToTimeMsg(t);
  msg.latitude = gp.latitude;
  msg.longitude = gp.longitude;

  return msg;
}

}  // namespace eufs::sim2::plugin
