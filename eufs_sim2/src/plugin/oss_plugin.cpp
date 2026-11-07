#include "eufs_sim2/plugin/oss_plugin.hpp"

#include "eufs_sim2/type/state.hpp"

namespace eufs::sim2::plugin {

geometry_msgs::msg::TwistWithCovarianceStamped OSSMessageConverter::Convert(
    const sensors::OSSVector &state, const time::Time &t) {
  geometry_msgs::msg::TwistWithCovarianceStamped msg;
  msg.header.stamp = time::TimeToTimeMsg(t);
  msg.header.frame_id = "oss";
  msg.twist.twist.linear.x = state(type::VehicleStateMember::_v_x);
  msg.twist.twist.linear.y = state(type::VehicleStateMember::_v_y);
  return msg;
}

}  // namespace eufs::sim2::plugin
