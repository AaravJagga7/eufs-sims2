#include "eufs_sim2/simulation.hpp"

namespace eufs::sim2::plugin {
  void Plugin::PostUpdate(const SimulationBase &sim) {
    if (sensor_failure_flags_[subnode_name_]) {
      return;
    }
    time::Time time = sim.GetCore().GetTime();
    if (time::HasTimeElapsed(time, previous_time_, frequency_)) {
      ThrottedPostUpdate(sim);
      previous_time_ = time;
    }
  }

  // Creating the Sensor Failure service
  void Plugin::CreateSensorFailureService(const std::string name) {
    std::string sensor_name = name.empty() ? subnode_name_ : name;
    std::string service_name = "/sensor_failure/" + sensor_name;

    std::string duration_param = sensor_name + ".failure_duration";
    if (!node_->has_parameter(duration_param)) {
      node_->declare_parameter<double>(duration_param, 5.0);
    }

    if (!sensor_failure_flags_.count(name)) {
      sensor_failure_flags_[name] = false;
    }
    sensor_failure_services_[service_name] = node_->create_service<std_srvs::srv::Trigger>(
          service_name,
          [this, name](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            this->TriggerSensorFailure(req, res, name);
          });

    RCLCPP_DEBUG(node_->get_logger(),
                "[%s] Failure service registered at %s",
                subnode_name_.c_str(), service_name.c_str());
  }

  // Creating the Sensor Failure function
  void Plugin::TriggerSensorFailure(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res,
    const std::string &name) {

    (void)req;

    bool &failed = sensor_failure_flags_[name];
    if (failed) {
      res->success = false;
      res->message = fmt::format("[{}] Already in failure state", name);
      return;
    }

    double duration = node_->get_parameter(name + ".failure_duration").as_double();
    failed = true;

    RCLCPP_INFO(node_->get_logger(),
                "[%s:%s] Failure service triggered for %.1f seconds",
                subnode_name_.c_str(), name.c_str(), duration);

    recovery_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(duration),
        [this, name]() {
          sensor_failure_flags_[name] = false;
          RCLCPP_INFO(node_->get_logger(),
                      "[%s:%s] Publishing resumed",
                      subnode_name_.c_str(), name.c_str());
          if (recovery_timer_) recovery_timer_->cancel();
        });

    res->success = true;
    res->message = fmt::format("[{}:{}] Failure triggered for {:.1f} seconds",
                              subnode_name_, name, duration);
  }

}  // namespace eufs::sim2::plugin
