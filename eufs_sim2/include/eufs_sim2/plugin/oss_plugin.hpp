#ifndef INCLUDE_EUFS_SIM2_PLUGIN_OSS_PLUGIN_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_OSS_PLUGIN_HPP_

#include <state_lib/state.hpp>
#include <state_lib/state_vars.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "eufs_sim2/noise/independent_gaussian.hpp"
#include "eufs_sim2/plugin/sensor_plugin.hpp"
#include "eufs_sim2/time/time.hpp"
#include "eufs_sim2/type/state.hpp"
#include "eufs_sim2/sensors/oss.hpp"

using VehicleState = eufs::sim2::type::VehicleState;

namespace eufs::sim2::plugin {

struct OSSMessageConverter {
  /**
   * @brief Convert internal twist representation to the ROS message
   * 
   * @param vec Speed sensor state vector
   * @param t Timestamp
   * @return Corresponding ROS message
   */
  static geometry_msgs::msg::TwistWithCovarianceStamped Convert(
    const sensors::OSSVector &vec, const time::Time &t);
};

using OSSPlugin = SensorPlugin<
  geometry_msgs::msg::TwistWithCovarianceStamped,
  OSSMessageConverter,
  noise::IndependentGaussianSampler,
  sensors::_oss_members,
  type::VehicleState::Vector>;

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_OSS_PLUGIN_HPP_
