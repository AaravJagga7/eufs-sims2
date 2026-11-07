#ifndef INCLUDE_EUFS_SIM2_PLUGIN_GNSS_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_GNSS_HPP_

#include <math.h>
#include <geodesy/utm.h>    // NOLINT
#include <geodesy/wgs84.h>  // NOLINT

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "eufs_sim2/core/base.hpp"
#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/sensors/gps.hpp"

namespace eufs::sim2::plugin {

class GNSSPlugin : public Plugin {
 public:
  // Inherit the constructor
  using Plugin::Plugin;

  /**
   * @brief Perform action at a set frequency
   * @param sim Simulation
   */
  void ThrottedPostUpdate(const SimulationBase &sim) override;

  /**
   * @brief Initialise the ROS interface of the plugin
   * @param node Node pointer for the ROS interface
   */
  void SetupROS(rclcpp::Node::SharedPtr node) override;

 private:
  /**
   * @brief convert state to gnss coordinates
   * @param sim Simulation
   */
  geographic_msgs::msg::GeoPoint GetGNSS(
    const eufs::sim2::core::VehicleState::Vector &vehiclestate);

 private:
  /**
   * @brief convert Gnss message to sensors_msgs::msg::NavSatFix message
   * @param gnss coordinates.
   * @param t Time stamp
   */
  sensor_msgs::msg::NavSatFix ConvertToMsg(geographic_msgs::msg::GeoPoint gp, time::Time t);

 private:
  // ROS
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_GNSS_HPP_
