#ifndef INCLUDE_EUFS_SIM2_PLUGIN_VEHICLE_STATE_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_VEHICLE_STATE_HPP_

#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "eufs_sim2/core/base.hpp"
#include "eufs_sim2/plugin/base.hpp"

namespace eufs::sim2::plugin {

class VehicleStatePlugin : public Plugin {
 public:
  // Inherit the constructor
  using Plugin::Plugin;

  /**
   * @brief Perform action at a set frequency
   */
  void ThrottedPostUpdate(const SimulationBase &sim) override;

  /**
   * @brief Initialise the ROS interface of the plugin
   */
  void SetupROS(rclcpp::Node::SharedPtr node) override;

 private:
  /**
   * @brief convert vehicle state to Odometry message
   * @param vs vehicle state.
   */
  nav_msgs::msg::Odometry ConvertToMsg(const type::VehicleState::Vector &vs, time::Time time);

 private:
  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_VEHICLE_STATE_HPP_
