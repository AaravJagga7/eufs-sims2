#include "eufs_sim2/noise/independent_gaussian.hpp"

namespace eufs::sim2::noise {

IndependentGaussianSampler::IndependentGaussianSampler(uint seed) : gen_(seed) {}

}  // namespace eufs::sim2::noise
