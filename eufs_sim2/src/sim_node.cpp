#include "eufs_sim2/sim_node.hpp"

#include <string>
#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "eufs_sim2/core/eufs_core.hpp"
#include "eufs_sim2/logger/logger.hpp"
#include "eufs_sim2/plugin/control_input.hpp"
#include "eufs_sim2/plugin/gnss.hpp"
#include "eufs_sim2/plugin/imu_plugin.hpp"
#include "eufs_sim2/plugin/oss_plugin.hpp"
#include "eufs_sim2/plugin/state_machine_plugin.hpp"
#include "eufs_sim2/plugin/track_changer.hpp"
#include "eufs_sim2/plugin/vehicle_state.hpp"
#include "eufs_sim2/plugin/wheel_speed.hpp"
#include "eufs_sim2/plugin/state_publisher.hpp"
#include "eufs_sim2/simulation.hpp"
#include "eufs_sim2/plugin/gt_transform.hpp"
#include "eufs_sim2/plugin/force_publisher.hpp"
#include "eufs_sim2/plugin/twist_publisher.hpp"
#include "eufs_sim2/plugin/cone_fusion.hpp"
#include "eufs_sim2/plugin/cone_collision_tracker.hpp"

namespace eufs::sim2 {

SimulationNode::SimulationNode() : rclcpp::Node("eufs_sim2") {
  std::string share_path = ament_index_cpp::get_package_share_directory("eufs_sim2");
  std::string eufs_master_path = std::getenv("EUFS_MASTER");

  // Register global logger
  eufs::sim2::logging::CreateLogger(eufs_master_path + "/log/sim2.log", "eufs_sim2", true);

  // General sim parameters
  auto frequency = declare_parameter<double>("frequency", 200);
  auto sim_step = declare_parameter<double>("sim_step", 1 / frequency);
  std::string core_params_path = declare_parameter<std::string>("core_params", "");
  sim_period_ = std::chrono::nanoseconds(static_cast<int>(1e9 * sim_step));

  // Initialise simulation node
  eufs::vehicle_models::Param param;
  plugin_yaml = std::filesystem::path(share_path) / "config" / "plugin_params.yaml";
  param.SetFromYaml(core_params_path);
  sim_ = std::make_shared<SimulationBase>(std::make_unique<core::EufsCore>(param));

  time_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

  // ROS Timer
  auto period = std::chrono::nanoseconds(static_cast<int>(1e9 / frequency));
  timer_ = create_wall_timer(period, std::bind(&SimulationNode::Loop, this));
}

std::unique_ptr<plugin::Plugin> SimulationNode::GetPlugin(std::string name) {
  if (name.starts_with("track_changer_plugin")) {
    return std::make_unique<eufs::sim2::plugin::TrackChangerPlugin>(name);
  }
  if (name.starts_with("control_input")) {
    return std::make_unique<eufs::sim2::plugin::ControlInputPlugin>(name);
  }
  if (name.starts_with("state_machine")) {
    return std::make_unique<eufs::sim2::plugin::StateMachinePlugin>(name);
  }
  if (name.starts_with("vehicle_state_plugin")) {
    return std::make_unique<eufs::sim2::plugin::VehicleStatePlugin>(name);
  }
  if (name.starts_with("wheel_speed_plugin")) {
    return std::make_unique<eufs::sim2::plugin::WheelSpeedPlugin>(name, "/wheel_speed");
  }
  if (name.starts_with("gnss_plugin")) {
    return std::make_unique<eufs::sim2::plugin::GNSSPlugin>(name);
  }
  if (name.starts_with("oss_plugin")) {
    return std::make_unique<eufs::sim2::plugin::OSSPlugin>(name, "/oss/data");
  }
  if (name.starts_with("imu_plugin")) {
    return std::make_unique<eufs::sim2::plugin::IMUPlugin>(name, "/imu/data");
  }
  if (name.starts_with("state_publisher")) {
    return std::make_unique<eufs::sim2::plugin::StatePublisherPlugin>(name);
  }
  if (name.starts_with("gt_transform")) {
    return std::make_unique<eufs::sim2::plugin::GTTransform>(name);
  }
  if (name.starts_with("force_publisher")) {
    return std::make_unique<eufs::sim2::plugin::ForcePublisher>(name);
  }
  if (name.starts_with("twist_publisher")) {
    return std::make_unique<eufs::sim2::plugin::TwistPublisher>(name);
  }
  if (name.starts_with("cone_fusion")) {
    return std::make_unique<eufs::sim2::plugin::ConeFusion>(name);
  }
  if (name.starts_with("cone_collision_tracker")) {
    return std::make_unique<eufs::sim2::plugin::ConeCollisionTracker>(name);
  }
  std::string error_message = "Unknown plugin: \"" + name + "\"";
  RCLCPP_ERROR(get_logger(), error_message.c_str());
  return NULL;
}

void SimulationNode::InitialisePlugins() {
  declare_parameter<std::vector<std::string>>("plugin_names");
  rclcpp::Parameter plugin_names_param = this->get_parameter("plugin_names");
  std::vector<std::string> plugin_names = plugin_names_param.get_value<std::vector<std::string>>();

  for (const std::string& name : plugin_names) {
    auto plugin = GetPlugin(name);
    if (!plugin) continue;
    plugin->SetupROS(shared_from_this());
    plugin->CreateSensorFailureService(name);
    sim_->RegisterPlugin(std::move(plugin));
  }
}

void SimulationNode::Loop() {
  // Step through sim
  sim_->Step(sim_period_);

  // Publish time
  time_pub_->publish(time::TimeToClockMsg(sim_->GetCore().GetTime()));
}

}  // namespace eufs::sim2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<eufs::sim2::SimulationNode>();

  //  Plugins can only be initialized once shared pointer to SimulationNode exists
  node->InitialisePlugins();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
