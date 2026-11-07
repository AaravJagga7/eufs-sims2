#include "eufs_sim2/plugin/imu_plugin.hpp"

#include <random>
#include <stdexcept>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "eufs_sim2/type/state.hpp"

namespace eufs::sim2::plugin {

std::vector<double> IMUMessageConverter::ang_drift;
std::vector<double> IMUMessageConverter::lin_drift;

void IMUPlugin::SetupROS(rclcpp::Node::SharedPtr node) {
  // Call base SetupROS function
  IMUPluginBase::SetupROS(node);

  // Handle IMU-specific parameters
  IMUMessageConverter::ang_drift = node_->declare_parameter<std::vector<double>>(
    fmt::format("{}angular_velocity_drift_vector", plugin_namespace));
  if (IMUMessageConverter::ang_drift.size() != 3) {
    throw std::invalid_argument("angular_velocity_drift_vector has the wrong size!");
  }

  IMUMessageConverter::lin_drift = node_->declare_parameter<std::vector<double>>(
    fmt::format("{}linear_acceleration_drift_vector", plugin_namespace));
  if (IMUMessageConverter::lin_drift.size() != 3) {
    throw std::invalid_argument("linear_acceleration_drift_vector has the wrong size!");
  }
}

sensor_msgs::msg::Imu IMUMessageConverter::Convert(
    const sensors::IMUVector &state, const time::Time &t) {

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = time::TimeToTimeMsg(t);
  msg.header.frame_id = "imu";

  double t_seconds = std::chrono::duration<double>(t).count();

  // Drift grows proportional to time
  using VSM = type::VehicleStateMember;

  msg.linear_acceleration.x = state(VSM::_a_x) + IMUMessageConverter::lin_drift[0] * t_seconds;
  msg.linear_acceleration.y = state(VSM::_a_y) + IMUMessageConverter::lin_drift[1] * t_seconds;
  msg.linear_acceleration.z = state(VSM::_a_z) + IMUMessageConverter::lin_drift[2] * t_seconds;
  msg.angular_velocity.x = state(VSM::_v_roll) + IMUMessageConverter::ang_drift[0] * t_seconds;
  msg.angular_velocity.y = state(VSM::_v_pitch) + IMUMessageConverter::ang_drift[1] * t_seconds;
  msg.angular_velocity.z = state(VSM::_v_yaw) + IMUMessageConverter::ang_drift[2] * t_seconds;

  tf2::Quaternion q;
  q.setRPY(state(type::VehicleStateMember::_roll), state(type::VehicleStateMember::_pitch),
           state(type::VehicleStateMember::_yaw));
  msg.orientation = tf2::toMsg(q);

  return msg;
}

}  // namespace eufs::sim2::plugin
