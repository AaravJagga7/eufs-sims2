#ifndef INCLUDE_EUFS_SIM2_CONCEPT_IMU_NOISE_SAMPLER_HPP_
#define INCLUDE_EUFS_SIM2_CONCEPT_IMU_NOISE_SAMPLER_HPP_

#include <state_lib/state_vars.hpp>

#include "eufs_sim2/sensors/imu.hpp"

namespace eufs::sim2::type {

template <typename Sampler>
concept IMUNoiseSampler = requires(
  Sampler sampler,
  const state::StateVariable<sensors::_imu_members> imu
) {
  sampler.template Sample(imu);
};  // NOLINT

}  // namespace eufs::sim2::type

#endif  // INCLUDE_EUFS_SIM2_CONCEPT_IMU_NOISE_SAMPLER_HPP_
