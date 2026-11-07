#ifndef INCLUDE_EUFS_SIM2_PLUGIN_BASE_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_BASE_HPP_

#include <fmt/core.h>

#include <chrono> // NOLINT
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "eufs_sim2/time/time.hpp"

namespace eufs::sim2 {

// Forward declaration;
class SimulationBase;

}  // namespace eufs::sim2

namespace eufs::sim2::plugin {

class Plugin {
 public:
  /**
   * @brief Virtual destructor
   */
  virtual ~Plugin() = default;

  /**
   * @brief construct plugin with name
   * @param name subnode name
   */
  explicit Plugin(std::string name) :
    subnode_name_(name), plugin_namespace("plugin." + name + ".") {}

  /**
   * @brief Initialise the ROS interface of the plugin
   */
  virtual void SetupROS(rclcpp::Node::SharedPtr) {}

  /**
   * @brief Perform action before simulation update step
   */
  virtual void PreUpdate(SimulationBase &) {}

  /**
   * @brief Runs at a throttled frequency
   */
  virtual void ThrottedPostUpdate(const SimulationBase &) {}

  /**
   * @brief Runs ThrottedPostUpdate at a set frequency
   */
  void PostUpdate(const SimulationBase &);

  /**
  * @brief Creates a ROS service for sensor failure
  */
  void CreateSensorFailureService(const std::string name);

  /**
  * @brief Callback for sensor failure trigger
  */
  void TriggerSensorFailure(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res, const std::string &sensor_name);

 protected:
  std::string subnode_name_;
  rclcpp::Node::SharedPtr node_;
  std::string plugin_namespace;
  double frequency_;
  eufs::sim2::time::Time previous_time_ = time::Time::zero();

  // Failure handling variables
  std::unordered_map<std::string, bool> sensor_failure_flags_;
  rclcpp::TimerBase::SharedPtr recovery_timer_;
  std::unordered_map<std::string,
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> sensor_failure_services_;
};

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_BASE_HPP_
