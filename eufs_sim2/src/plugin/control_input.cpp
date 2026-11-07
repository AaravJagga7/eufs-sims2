#include "eufs_sim2/plugin/control_input.hpp"

#include <functional>
#include <utility>

#include "eufs_sim2/simulation.hpp"

namespace eufs::sim2::plugin {

void ControlInputPlugin::SetCommand(type::ControlInput cmd) {
  cmd_ = cmd;
  new_cmd_ = true;
}

void ControlInputPlugin::PreUpdate(SimulationBase &sim) {
  // Only update the sim command if a new command has actually been recieved
  if (new_cmd_) {
    sim.SetCommand(cmd_);
    new_cmd_ = false;
  }
}

void ControlInputPlugin::SetupROS(rclcpp::Node::SharedPtr node) {
  node_ = node->create_sub_node(subnode_name_);
  frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 200);

  // Create ROS subscribers
  cmd_sub_ = node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/cmd", 1, std::bind(&ControlInputPlugin::CommandCallback, this, std::placeholders::_1));
}

void ControlInputPlugin::CommandCallback(const ackermann_msgs::msg::AckermannDriveStamped &msg) {
  SetCommand(type::ToControlInput(msg));
}

}  // namespace eufs::sim2::plugin
