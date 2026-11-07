#ifndef INCLUDE_EUFS_SIM2_CONCEPT_OSS_NOISE_SAMPLER_HPP_
#define INCLUDE_EUFS_SIM2_CONCEPT_OSS_NOISE_SAMPLER_HPP_

#include <state_lib/state_vars.hpp>

#include "eufs_sim2/sensors/oss.hpp"

namespace eufs::sim2::type {

template <typename Sampler>
concept OSSNoiseSampler = requires(
  Sampler sampler,
  const state::StateVariable<sensors::_oss_members> oss
) {
  sampler.template Sample(oss);
};  // NOLINT

}  // namespace eufs::sim2::type

#endif  // INCLUDE_EUFS_SIM2_CONCEPT_OSS_NOISE_SAMPLER_HPP_
