#ifndef INCLUDE_EUFS_SIM2_CONCEPT_WHEEL_SPEED_NOISE_SAMPLER_HPP_
#define INCLUDE_EUFS_SIM2_CONCEPT_WHEEL_SPEED_NOISE_SAMPLER_HPP_

#include <state_lib/state_vars.hpp>

#include "eufs_sim2/sensors/wheel_speeds.hpp"

namespace eufs::sim2::type {

template <typename Sampler>
concept WheelSpeedNoiseSampler = requires(
  Sampler sampler,
  const state::StateVariable<sensors::_wheel_speed_members> wheel_speeds
) {
  sampler.template Sample(wheel_speeds);
};  // NOLINT

}  // namespace eufs::sim2::type

#endif  // INCLUDE_EUFS_SIM2_CONCEPT_WHEEL_SPEED_NOISE_SAMPLER_HPP_
