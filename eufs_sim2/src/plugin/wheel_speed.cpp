#include "eufs_sim2/plugin/wheel_speed.hpp"

#include "eufs_sim2/sensors/wheel_speeds.hpp"

namespace eufs::sim2::plugin {

eufs_msgs::msg::WheelSpeedsStamped WheelSpeedsMessageConverter::Convert(
    const sensors::WheelSpeedsVector &ws, const time::Time &time) {
  eufs_msgs::msg::WheelSpeedsStamped msg;
  msg.header.stamp = time::TimeToTimeMsg(time);
  msg.header.frame_id = "wheel_speed";
  msg.speeds.lf_speed = ws(sensors::WheelSpeedsMember::_fl);
  msg.speeds.rf_speed = ws(sensors::WheelSpeedsMember::_fr);
  msg.speeds.lb_speed = ws(sensors::WheelSpeedsMember::_rl);
  msg.speeds.rb_speed = ws(sensors::WheelSpeedsMember::_rr);
  msg.speeds.steering = ws(sensors::WheelSpeedsMember::_steering);
  return msg;
}
}  // namespace eufs::sim2::plugin
