#ifndef INCLUDE_EUFS_SIM2_PLUGIN_CONTROL_INPUT_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_CONTROL_INPUT_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include "eufs_sim2/core/base.hpp"
#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/type/control.hpp"

namespace eufs::sim2::plugin {

class ControlInputPlugin : public Plugin {
 public:
  // Inherit the constructor
  using Plugin::Plugin;

  /**
   * @brief Set latest control command
   * @param cmd Command to send to car
   */
  inline void SetCommand(type::ControlInput cmd);

  /**
   * @brief Run the plugin
   * @param sim Simulation
   */
  void PreUpdate(SimulationBase &sim) override;

  /**
   * @brief Initialise the ROS interface of the plugin
   * @param node Node handler for the ROS interface
   */
  void SetupROS(rclcpp::Node::SharedPtr node) override;

 private:
  /**
   * @brief Command message callback
   * @param msg AckermannDriveStamped command message
   */
  void CommandCallback(const ackermann_msgs::msg::AckermannDriveStamped &msg);

 private:
  type::ControlInput cmd_;
  bool new_cmd_ = false;

  // ROS
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
};

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_CONTROL_INPUT_HPP_
