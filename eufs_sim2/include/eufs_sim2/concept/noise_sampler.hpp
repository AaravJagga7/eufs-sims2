#ifndef INCLUDE_EUFS_SIM2_CONCEPT_NOISE_SAMPLER_HPP_
#define INCLUDE_EUFS_SIM2_CONCEPT_NOISE_SAMPLER_HPP_

#include "eufs_sim2/concept/imu_noise_sampler.hpp"
#include "eufs_sim2/concept/wheel_speed_noise_sampler.hpp"
#include "eufs_sim2/concept/oss_noise_sampler.hpp"

namespace eufs::sim2::type {

template <typename Sampler>
concept NoiseSampler = (
    IMUNoiseSampler<Sampler> &&
    OSSNoiseSampler<Sampler> &&
    WheelSpeedNoiseSampler<Sampler>);

}  // namespace eufs::sim2::type

#endif  // INCLUDE_EUFS_SIM2_CONCEPT_NOISE_SAMPLER_HPP_
