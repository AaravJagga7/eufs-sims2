#include "eufs_sim2/plugin/vehicle_state.hpp"

#include <functional>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "eufs_sim2/simulation.hpp"
#include "eufs_sim2/time/time.hpp"
#include "eufs_sim2/type/state.hpp"

namespace eufs::sim2::plugin {

void VehicleStatePlugin::ThrottedPostUpdate(const SimulationBase &sim) {
  time::Time now = sim.GetCore().GetTime();

  type::VehicleState::Vector state_data;
  sim.GetCore().GetState(state_data);
  publisher_->publish(ConvertToMsg(state_data, now));
}

void VehicleStatePlugin::SetupROS(rclcpp::Node::SharedPtr node) {
  node_ = node->create_sub_node(subnode_name_);
  publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/plugin/" + subnode_name_ + "/ground_truth/state", 10);
  frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 200);
  RCLCPP_INFO(node_->get_logger(), "Vehicle State Plugin Initialised!");
}

nav_msgs::msg::Odometry VehicleStatePlugin::ConvertToMsg(
    const type::VehicleState::Vector &vs, time::Time time) {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = time::TimeToTimeMsg(time);
  msg.header.frame_id = "map";
  msg.child_frame_id = "base_footprint";
  msg.pose.pose.position.x = vs(type::VehicleStateMember::_x);
  msg.pose.pose.position.y = vs(type::VehicleStateMember::_y);
  msg.pose.pose.position.z = vs(type::VehicleStateMember::_z);
  tf2::Quaternion q;
  q.setRPY(vs(type::VehicleStateMember::_roll), vs(type::VehicleStateMember::_pitch),
           vs(type::VehicleStateMember::_yaw));
  msg.pose.pose.orientation = tf2::toMsg(q);
  msg.twist.twist.linear.x = vs(type::VehicleStateMember::_v_x);
  msg.twist.twist.linear.y = vs(type::VehicleStateMember::_v_y);
  msg.twist.twist.linear.z = vs(type::VehicleStateMember::_v_z);
  msg.twist.twist.angular.x = vs(type::VehicleStateMember::_v_roll);
  msg.twist.twist.angular.y = vs(type::VehicleStateMember::_v_pitch);
  msg.twist.twist.angular.z = vs(type::VehicleStateMember::_v_yaw);
  return msg;
}
}  // namespace eufs::sim2::plugin
