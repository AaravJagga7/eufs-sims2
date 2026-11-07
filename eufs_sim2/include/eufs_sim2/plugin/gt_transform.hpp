#ifndef INCLUDE_EUFS_SIM2_PLUGIN_GT_TRANSFORM_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_GT_TRANSFORM_HPP_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "eufs_sim2/core/base.hpp"
#include "eufs_sim2/plugin/base.hpp"

using VehicleState = eufs::sim2::type::VehicleState;


namespace eufs::sim2::plugin {
class GTTransform : public Plugin {
 public:
  // Inherit the constructor
  using Plugin::Plugin;

  /**
   * @brief Perform action at a set frequency
   * @param sim Simulation
   */
  void ThrottedPostUpdate(const SimulationBase &sim) override;

  /**
   * @brief Initialise the ROS interface of the plugin
   * @param node Node pointer for the ROS interface
   */
  void SetupROS(rclcpp::Node::SharedPtr node) override;

 private:
   /**
   * @brief convert vehicle state data to geometry_msgs::msg::TransformStamped message
   * @param state vehicle state.
   * @param t Time stamp
   */
  geometry_msgs::msg::TransformStamped ConvertToMsg(
    const type::VehicleState::Vector &state, time::Time t);

 private:
  // ROS
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string vehicle_frame_id;
};

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_GT_TRANSFORM_HPP_
