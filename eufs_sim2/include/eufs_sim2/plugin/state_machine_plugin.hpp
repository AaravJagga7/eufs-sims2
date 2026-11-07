#ifndef INCLUDE_EUFS_SIM2_PLUGIN_STATE_MACHINE_PLUGIN_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_STATE_MACHINE_PLUGIN_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "eufs_msgs/srv/set_mission.hpp"
#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/state_machine.hpp"

namespace eufs::sim2::plugin {
  using AMIState = eufs::sim2::state_machine::AMIState;

class StateMachinePlugin : public Plugin {
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
   * @brief Service for setting the EBS
   */
  void SetEbs(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Service callback for setting the mission
   */
  void SetMission(const std::shared_ptr<eufs_msgs::srv::SetMission::Request> request,
                  std::shared_ptr<eufs_msgs::srv::SetMission::Response> response);

  /**
   * @brief Service callback for resetting the sim
   */
  void Reset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Service for setting GO flag
   */
  void SetGo(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Service for setting ENFORCE flag
   */
  void SetEnforce(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Mission finished callback to be set once the mission is finished
   *
   */
  void MissionFinishedCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Driving_flag topic callback for setting GO flag
   *
   */
  void DrivingFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Helper function for setting the control input to zero
   */
  void ZeroControlInput(SimulationBase &sim);  // NOLINT

  /**
   * @brief Helper function for setting the control input to EBS
   */
  void EBSControlInput(SimulationBase &sim);  // NOLINT

   /**
   * @brief Helper function for resetting flag variables in state machine plugin
   */
  void ResetFlags();

 private:
  AMIState ami_state_ = AMIState::NOT_SELECTED;
  bool ebs_active_ = false;
  bool mission_finished_ = false;
  bool reset_ = false;
  bool go_ = false;
  bool enforce_ = false;

  // ROS
  // services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ebs_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enforce_service_;
  rclcpp::Service<eufs_msgs::srv::SetMission>::SharedPtr mission_service_;

  // subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_finished_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driving_flag_sub_;
};

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_STATE_MACHINE_PLUGIN_HPP_
