#include "eufs_sim2/plugin/state_publisher.hpp"

namespace eufs::sim2::plugin {

void StatePublisherPlugin::SetupROS(rclcpp::Node::SharedPtr node) {
  node_ = node->create_sub_node(subnode_name_);
  frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 200);

  state_pub_ = node_->create_publisher<eufs_msgs::msg::CanState>(
      "/plugin/" + subnode_name_ + "/ros_can/state", 1);
  state_pub_str_ = node_->create_publisher<std_msgs::msg::String>(
      "/plugin/" + subnode_name_ + "/ros_can/state_str", 1);
}

eufs_msgs::msg::CanState StatePublisherPlugin::ConvertToCanMsg(state_machine::AMIState amiState,
    state_machine::ASState asState) {

  eufs_msgs::msg::CanState msg;

  msg.ami_state = ami_state_translation[amiState];
  msg.as_state = as_state_translation[asState];

  return msg;
}

std_msgs::msg::String StatePublisherPlugin::ConvertToCanStr(
    state_machine::AMIState amiState,
    state_machine::ASState asState) {

  std_msgs::msg::String msg;

  msg.data = "AMIState: " + state_machine::StateMachine::GetEnumString(amiState);
  msg.data += "  ASState: " + state_machine::StateMachine::GetEnumString(asState);
  return msg;
}

void StatePublisherPlugin::ThrottedPostUpdate(const SimulationBase &sim) {
  state_machine::AMIState amiState = sim.GetStateMachine().GetAMIState();
  state_machine::ASState asState = sim.GetStateMachine().GetASState();

  eufs_msgs::msg::CanState stateMsg = ConvertToCanMsg(amiState, asState);
  std_msgs::msg::String stateMsgStr = ConvertToCanStr(amiState, asState);

  state_pub_->publish(stateMsg);
  state_pub_str_->publish(stateMsgStr);
}
}  // namespace eufs::sim2::plugin
