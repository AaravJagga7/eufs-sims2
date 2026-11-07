#ifndef INCLUDE_EUFS_SIM2_NOISE_INDEPENDENT_GAUSSIAN_HPP_
#define INCLUDE_EUFS_SIM2_NOISE_INDEPENDENT_GAUSSIAN_HPP_

#include <random>

#include <state_lib/state.hpp>
#include <state_lib/state_vars.hpp>
#include <state_lib/state_vector.hpp>

namespace eufs::sim2::noise {

class IndependentGaussianSampler {
 public:
  explicit IndependentGaussianSampler(uint seed = std::random_device {}());

  /**
   * @brief Sample noise
   * 
   * @param state The mean of the sampling distribution
   * @return sample from distribution
   */
  template <state::StateVars vars>
  state::StateVector<vars, double> Sample(const state::StateVariable<vars> &state) {
    state::StateVector<vars, double> noisy_data;

    std::invoke([&]<state::StateElement ...els>(const state::StateVars<els...> &) {
      ( (noisy_data(state::StateVars<els>{})
        = std::normal_distribution<double>(
            state.state(state::StateVars<els>{}),
            std::sqrt(state.covariance(state::StateVars<els>{}))) (gen_) ), ...);
    }, vars);

    return noisy_data;
  }

 private:
  std::mt19937 gen_;
};

}  // namespace eufs::sim2::noise

#endif  // INCLUDE_EUFS_SIM2_NOISE_INDEPENDENT_GAUSSIAN_HPP_
