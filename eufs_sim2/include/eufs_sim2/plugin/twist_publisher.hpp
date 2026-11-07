#ifndef INCLUDE_EUFS_SIM2_PLUGIN_TWIST_PUBLISHER_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_TWIST_PUBLISHER_HPP_

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/simulation.hpp"

namespace eufs::sim2::plugin {

class TwistPublisher : public Plugin {
 public:
  // Inherit the constructor
  using Plugin::Plugin;

  /**
   * @brief Publish twist message
   * @param sim Simulation
  */
  void ThrottedPostUpdate(const SimulationBase &sim) override;

  /**
   * @brief Initialise the ROS interface of the plugin
   * @param node Node handler for the ROS interface
  */
  void SetupROS(rclcpp::Node::SharedPtr node) override;

 private:
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
};
}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_TWIST_PUBLISHER_HPP_
