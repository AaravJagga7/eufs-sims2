#ifndef INCLUDE_EUFS_SIM2_SIM_NODE_HPP_
#define INCLUDE_EUFS_SIM2_SIM_NODE_HPP_

#include <chrono>  // NOLINT
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "eufs_sim2/core/base.hpp"
#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/simulation.hpp"

namespace eufs::sim2 {

class SimulationNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct new Simulation Node object.
   */
  SimulationNode();

  /**
   * @brief Initialise plugins for the simulation
   */
  void InitialisePlugins();

 private:
  /**
   * @brief Main simulation loop
   */
  void Loop();

  /**
   * @brief Return an instance of a plugin given the name
   * @return
   */
  std::unique_ptr<plugin::Plugin> GetPlugin(std::string);

 private:
  // ROS publisher
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr time_pub_;

  // ROS timer
  rclcpp::TimerBase::SharedPtr timer_;

 private:
  std::chrono::nanoseconds sim_period_;

 private:
  std::shared_ptr<SimulationBase> sim_;
  std::string plugin_yaml;
};

}  // namespace eufs::sim2

#endif  // INCLUDE_EUFS_SIM2_SIM_NODE_HPP_
