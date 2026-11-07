#include "eufs_sim2/plugin/twist_publisher.hpp"

#include "eufs_sim2/type/state.hpp"

namespace eufs::sim2::plugin {

  void TwistPublisher::SetupROS(rclcpp::Node::SharedPtr node) {
    node_ = node->create_sub_node(subnode_name_);
    frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 20);

    twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/plugin/" + subnode_name_ + "/twist", 1);
  }

  void TwistPublisher::ThrottedPostUpdate(const SimulationBase &sim) {
    geometry_msgs::msg::TwistWithCovarianceStamped msg;

    msg.header.stamp = time::TimeToTimeMsg(sim.GetCore().GetTime());
    msg.header.frame_id = "base_footprint";

    type::VehicleState::Vector car_state;
    sim.GetCore().GetState(car_state);

    msg.twist.twist.linear.x = car_state[type::VehicleStateMember::_v_x];
    msg.twist.twist.linear.y = car_state[type::VehicleStateMember::_v_y];
    msg.twist.twist.linear.z = car_state[type::VehicleStateMember::_v_z];

    msg.twist.twist.angular.x = car_state[type::VehicleStateMember::_v_pitch];
    msg.twist.twist.angular.y = car_state[type::VehicleStateMember::_v_yaw];
    msg.twist.twist.angular.z = car_state[type::VehicleStateMember::_v_roll];

    msg.twist.covariance = {
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0
    };

    twist_pub_->publish(msg);
  }

}  // namespace eufs::sim2::plugin
