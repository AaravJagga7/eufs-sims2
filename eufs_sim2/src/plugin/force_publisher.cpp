#include "eufs_sim2/plugin/force_publisher.hpp"

#include "eufs_sim2/sensors/vehicle_forces.hpp"

namespace eufs::sim2::plugin {
  void ForcePublisher::SetupROS(rclcpp::Node::SharedPtr node) {
    node_ = node->create_sub_node(subnode_name_);
    frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 20);

    force_pub_ = node_->create_publisher<eufs_msgs::msg::CarForces>(
        "/plugin/" + subnode_name_ + "/car_forces", 1);
  }

  void ForcePublisher::ThrottedPostUpdate(const SimulationBase &sim) {
    eufs_msgs::msg::CarForces msg;
    sensors::VehicleForces vf = sim.GetCore().GetVehicleForces();

    msg.front_left_lateral = vf.state(sensors::VehicleForcesMember::_fl_la);
    msg.front_left_longitudinal = vf.state(sensors::VehicleForcesMember::_fl_lo);
    msg.front_left_vertical = vf.state(sensors::VehicleForcesMember::_fl_ve);

    msg.front_right_lateral = vf.state(sensors::VehicleForcesMember::_fr_la);
    msg.front_right_longitudinal = vf.state(sensors::VehicleForcesMember::_fr_lo);
    msg.front_right_vertical = vf.state(sensors::VehicleForcesMember::_fr_ve);

    msg.rear_left_lateral = vf.state(sensors::VehicleForcesMember::_rl_la);
    msg.rear_left_longitudinal = vf.state(sensors::VehicleForcesMember::_rl_lo);
    msg.rear_left_vertical = vf.state(sensors::VehicleForcesMember::_rl_ve);

    msg.rear_right_lateral = vf.state(sensors::VehicleForcesMember::_rr_la);
    msg.rear_right_longitudinal = vf.state(sensors::VehicleForcesMember::_rr_lo);
    msg.rear_right_vertical = vf.state(sensors::VehicleForcesMember::_rr_ve);

    force_pub_->publish(msg);
  }
}  // namespace eufs::sim2::plugin
