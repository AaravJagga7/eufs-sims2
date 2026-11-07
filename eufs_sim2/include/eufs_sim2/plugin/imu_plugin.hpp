#ifndef INCLUDE_EUFS_SIM2_PLUGIN_IMU_PLUGIN_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_IMU_PLUGIN_HPP_

#include <vector>

#include <state_lib/state.hpp>
#include <state_lib/state_vars.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include "eufs_sim2/noise/independent_gaussian.hpp"
#include "eufs_sim2/plugin/sensor_plugin.hpp"
#include "eufs_sim2/time/time.hpp"
#include "eufs_sim2/type/state.hpp"
#include "eufs_sim2/sensors/imu.hpp"

using VehicleState = eufs::sim2::type::VehicleState;

namespace eufs::sim2::plugin {

struct IMUMessageConverter {
  static std::vector<double> ang_drift;
  static std::vector<double> lin_drift;

  /**
   * @brief Convert internal IMU state representation to the ROS message
   * 
   * @param vec IMU state vector
   * @param t Timestamp
   * @return Corresponding ROS message
   */
  static sensor_msgs::msg::Imu Convert(const sensors::IMUVector &vec, const time::Time &t);
};

// Define base type for cleaner inheritance
using IMUPluginBase = SensorPlugin<
sensor_msgs::msg::Imu,
IMUMessageConverter,
noise::IndependentGaussianSampler,
sensors::_imu_members,
type::VehicleState::Vector>;

class IMUPlugin : public IMUPluginBase {
 public:
  // Inherit the constructor
  using IMUPluginBase::SensorPlugin;

  /**
   * @brief Initialise the ROS interface of the plugin
   * @param node Node handler for the ROS interface
   */
  void SetupROS(rclcpp::Node::SharedPtr node) override;
};

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_IMU_PLUGIN_HPP_
