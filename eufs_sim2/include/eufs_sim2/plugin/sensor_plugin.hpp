#ifndef INCLUDE_EUFS_SIM2_PLUGIN_SENSOR_PLUGIN_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_SENSOR_PLUGIN_HPP_

#include <fmt/core.h>

#include <string>
#include <array>
#include <vector>
#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <state_lib/state_vars.hpp>
#include <state_lib/state_mask.hpp>

#include "eufs_sim2/core/base.hpp"
#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/simulation.hpp"
#include "eufs_sim2/concept/message_converter.hpp"
#include "eufs_sim2/concept/noise_sampler.hpp"

namespace eufs::sim2::plugin {

template<
  typename MessageType,
  type::MessageConverter MessageConverter,
  type::NoiseSampler NoiseSampler,
  state::StateVars sensor_vars,
  typename SupersetVector>
class SensorPlugin : public Plugin {
  using SensorState = state::StateVariable<sensor_vars, double>;
  constexpr static auto covariance_size =
    state::GetVarsNumber(sensor_vars) * state::GetVarsNumber(sensor_vars);

 public:
  explicit SensorPlugin(const std::string &name, const std::string &topic_name)
    : Plugin(name), topic_name_(topic_name) {}

  void ThrottedPostUpdate(const SimulationBase &sim) {
    SupersetVector superset;
    sim.GetCore().GetState(superset);
    time::Time time = sim.GetCore().GetTime();

    sensor_data_.state = superset(state::StateMask(sensor_vars));
    typename SensorState::Vector vec = noise_gen_->Sample(sensor_data_);

    publisher_->publish(MessageConverter::Convert(std::move(vec), time));
  }

  void SetupROS(rclcpp::Node::SharedPtr node) {
    node_ = node->create_sub_node(subnode_name_);

    publisher_ = node_->create_publisher<MessageType>(
      fmt::format("/plugin/{}{}", subnode_name_, topic_name_), 10);
    frequency_ = node_->declare_parameter<double>(
      fmt::format("{}frequency", plugin_namespace), 200);

    auto noise_covariance = node_->declare_parameter<std::vector<double>>(
      fmt::format("{}noise_covariance", plugin_namespace));
    if (noise_covariance.size() != covariance_size) {
      throw std::invalid_argument("Initial state covariance has the wrong size!");
    }
    std::copy_n(noise_covariance.begin(), covariance_size, noise_covariance_.begin());
    sensor_data_.covariance = noise_covariance_;

    const int seed = node_->declare_parameter<int>(
      fmt::format("{}noise_seed", plugin_namespace), 0);
    noise_gen_ = std::make_unique<NoiseSampler>(NoiseSampler(seed));
    RCLCPP_INFO(node_->get_logger(), fmt::format("{} plugin initialised!", subnode_name_).c_str());
  }

 private:
  SensorState sensor_data_;

  std::string topic_name_;
  std::array<double, covariance_size> noise_covariance_;
  std::unique_ptr<NoiseSampler> noise_gen_;
  rclcpp::Publisher<MessageType>::SharedPtr publisher_;
};

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_SENSOR_PLUGIN_HPP_
