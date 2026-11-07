#include "eufs_sim2/plugin/state_machine_plugin.hpp"

#include <functional>

#include "eufs_sim2/simulation.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace eufs::sim2::plugin {

void StateMachinePlugin::PreUpdate(SimulationBase &sim) {
  if (ebs_active_) {
    sim.GetStateMachine().TriggerEBS();
  }

  if (mission_finished_) {
    sim.GetStateMachine().FinishMission();
    mission_finished_ = false;
  }

  if (reset_) {
    sim.GetCore().Reset();
    sim.GetStateMachine().ResetState();
    ResetFlags();
  }

  if (go_) {
    sim.GetStateMachine().StartDriving();
  }

  if (ami_state_ != AMIState::NOT_SELECTED) {
    sim.GetStateMachine().SetMission(ami_state_);
  }

  sensors::WheelSpeeds::Vector car_state;
  sim.GetStateMachine().UpdateStatus(sim.GetCore().GetTime(), sim.GetCore().GetState(car_state));

  switch (sim.GetStateMachine().GetASState()) {
    case eufs::sim2::state_machine::ASState::OFF:
      if (ebs_active_) {
        ResetFlags();
      }
      if (enforce_) {
        ZeroControlInput(sim);
      }
      break;

    case eufs::sim2::state_machine::ASState::DRIVING:
      break;

    case eufs::sim2::state_machine::ASState::EMERGENCY_BRAKE:
      EBSControlInput(sim);
      break;

    default:
      ZeroControlInput(sim);
      break;
  }
}

void StateMachinePlugin::SetupROS(rclcpp::Node::SharedPtr node) {
  node_ = node->create_sub_node(subnode_name_);
  frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 200);

  enforce_ = node_->declare_parameter<bool>(plugin_namespace + "enforce", true);

  // Create ROS subscribers
  mission_finished_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/complete_mission_flag", 1,
      std::bind(&StateMachinePlugin::MissionFinishedCallback, this, std::placeholders::_1));

  driving_flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/driving_flag", 1,
      std::bind(&StateMachinePlugin::DrivingFlagCallback, this, std::placeholders::_1));

  // Create ROS services
  ebs_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/ebs",
      std::bind(&StateMachinePlugin::SetEbs, this, std::placeholders::_1, std::placeholders::_2));

  mission_service_ = node_->create_service<eufs_msgs::srv::SetMission>(
      "/set_mission", std::bind(&StateMachinePlugin::SetMission, this, std::placeholders::_1,
                                std::placeholders::_2));

  reset_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/reset",
      std::bind(&StateMachinePlugin::Reset, this, std::placeholders::_1, std::placeholders::_2));

  go_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/go", std::bind(&StateMachinePlugin::SetGo, this, std::placeholders::_1,
                       std::placeholders::_2));

  enforce_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/toggle_enforce", std::bind(&StateMachinePlugin::SetEnforce, this, std::placeholders::_1,
                       std::placeholders::_2));

  RCLCPP_INFO(node_->get_logger(), "State machine plugin initialised!");
}

void StateMachinePlugin::SetEbs(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  ebs_active_ = true;
  response->success = true;
}

void StateMachinePlugin::SetMission(
    const std::shared_ptr<eufs_msgs::srv::SetMission::Request> request,
    std::shared_ptr<eufs_msgs::srv::SetMission::Response> response) {
  ami_state_ = static_cast<AMIState>(request->mission);
  response->success = true;
}

void StateMachinePlugin::Reset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  reset_ = true;
  response->success = true;
}

void StateMachinePlugin::SetGo(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  go_ = true;
  response->success = true;
}

void StateMachinePlugin::SetEnforce(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  enforce_ = !enforce_;
  response->success = true;
}

void StateMachinePlugin::MissionFinishedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  mission_finished_ = msg->data;
}

void StateMachinePlugin::DrivingFlagCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  go_ = msg->data;
}

void StateMachinePlugin::ZeroControlInput(SimulationBase &sim) {
  eufs::sim2::type::ControlInput control_input;
  control_input.acceleration = 0.0;
  control_input.steering_angle = 0.0;

  sim.SetCommand(control_input);
}

void StateMachinePlugin::EBSControlInput(SimulationBase &sim) {
  eufs::sim2::type::ControlInput control_input;
  control_input.acceleration = -10.0;
  control_input.steering_angle = 0.0;

  sim.SetCommand(control_input);
}

void StateMachinePlugin::ResetFlags() {
  reset_ = false;
  ebs_active_ = false;
  mission_finished_ = false;
  go_ = false;
  ami_state_ = AMIState::NOT_SELECTED;
  }
}  // namespace eufs::sim2::plugin
