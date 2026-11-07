#ifndef INCLUDE_EUFS_SIM2_PLUGIN_FORCE_PUBLISHER_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_FORCE_PUBLISHER_HPP_

#include <eufs_msgs/msg/car_forces.hpp>

#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/simulation.hpp"

namespace eufs::sim2::plugin {
class ForcePublisher : public Plugin {
 public:
  // Inherit the constructor
  using Plugin::Plugin;

  /**
   * @brief Publish car forces (for now just wheel forces)
   * @param sim Simulation
  */
  void ThrottedPostUpdate(const SimulationBase &sim) override;

  /**
   * @brief Initialise the ROS interface of the plugin
   * @param node Node handler for the ROS interface
  */
  void SetupROS(rclcpp::Node::SharedPtr node) override;

 private:
  rclcpp::Publisher<eufs_msgs::msg::CarForces>::SharedPtr force_pub_;
};
}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_FORCE_PUBLISHER_HPP_
