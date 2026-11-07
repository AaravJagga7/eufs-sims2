#ifndef INCLUDE_EUFS_SIM2_PLUGIN_TRACK_CHANGER_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_TRACK_CHANGER_HPP_

#include <memory>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/srv/set_string.hpp>
#include <eufs_msgs/srv/set_track.hpp>
#include <eufs_msgs/srv/get_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include "eufs_sim2/core/base.hpp"
#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/simulation.hpp"
#include "eufs_sim2/type/track.hpp"

namespace eufs::sim2::plugin {

class TrackChangerPlugin : public Plugin {
 public:
  // Inherit the constructor
  using Plugin::Plugin;

  /**
   * @brief Run the plugin
   * @param sim Simulation
   */
  void PreUpdate(SimulationBase &sim) override;

  /**
   * @brief Initialise the ROS interface of the plugin
   * @param node Node handler for the ROS interface
   */
  void SetupROS(rclcpp::Node::SharedPtr node) override;

 private:
  /**
   * @brief Handle client request of file path
   * @param req Request (containing file path to track)
   * @param res Response
   */
  void TrackFromFile(std::shared_ptr<eufs_msgs::srv::SetString::Request> req,
                     std::shared_ptr<eufs_msgs::srv::SetString::Response> res);

  /**
   * @brief Handle client request of track message
   * @param req Request (containing ConeArrayWithCovariance of track)
   * @param res Response
   */
  void TrackFromMsg(std::shared_ptr<eufs_msgs::srv::SetTrack::Request> req,
                    std::shared_ptr<eufs_msgs::srv::SetTrack::Response> res);

  /**
   * @brief Handle client request of getting all available maps
   * @param req Empty
   * @param res Response (String Array)
   */
  void GetMap(std::shared_ptr<eufs_msgs::srv::GetMap::Request> req,
                    std::shared_ptr<eufs_msgs::srv::GetMap::Response> res);

 private:
  bool track_changed = false;  // Set to false as service_file_ changes map
  type::Track track_;
  type::Track ground_truth_track_;

  // ROS
  rclcpp::Service<eufs_msgs::srv::SetString>::SharedPtr service_file_;
  rclcpp::Service<eufs_msgs::srv::SetTrack>::SharedPtr service_msg_;
  rclcpp::Service<eufs_msgs::srv::GetMap>::SharedPtr service_get_map_;

  // Remove cones
  int remove_cones_num_;

  // Change cone colors
  int color_change_to_orange_;
  int color_change_to_yellow_;
  int color_change_to_blue_;

  // Shift cones
  int shift_cones_num_;
  double shift_cones_min_radius_;
  double shift_cones_max_radius_;
};
}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_TRACK_CHANGER_HPP_
