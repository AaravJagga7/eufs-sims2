#ifndef INCLUDE_EUFS_SIM2_PLUGIN_WHEEL_SPEED_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_WHEEL_SPEED_HPP_

#include <state_lib/state.hpp>
#include <state_lib/state_vars.hpp>

#include <eufs_msgs/msg/wheel_speeds_stamped.hpp>

#include "eufs_sim2/noise/independent_gaussian.hpp"
#include "eufs_sim2/plugin/sensor_plugin.hpp"
#include "eufs_sim2/sensors/wheel_speeds.hpp"

namespace eufs::sim2::plugin {

struct WheelSpeedsMessageConverter {
  /**
   * @brief Convert internal wheel speeds representation to the ROS message
   * 
   * @param vec Wheel speeds vector
   * @param t Timestamp
   * @return Corresponding ROS message
   */
  static eufs_msgs::msg::WheelSpeedsStamped Convert(
    const sensors::WheelSpeedsVector &vec, const time::Time &t);
};

using WheelSpeedPlugin = SensorPlugin<
  eufs_msgs::msg::WheelSpeedsStamped,
  WheelSpeedsMessageConverter,
  noise::IndependentGaussianSampler,
  sensors::_wheel_speed_members,
  sensors::WheelSpeeds::Vector>;

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_WHEEL_SPEED_HPP_
