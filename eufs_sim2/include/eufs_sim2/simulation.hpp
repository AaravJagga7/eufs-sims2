#ifndef INCLUDE_EUFS_SIM2_SIMULATION_HPP_
#define INCLUDE_EUFS_SIM2_SIMULATION_HPP_

#include <functional>
#include <memory>
#include <utility>
#include <vector>


#include <map_lib/type/map.hpp>

#include "eufs_sim2/core/base.hpp"
#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/state_machine.hpp"
#include "eufs_sim2/time/time.hpp"
#include "eufs_sim2/type/track.hpp"

namespace eufs::sim2 {

class SimulationBase {
  using Duration = eufs::sim2::time::Duration;
  using Plugin = eufs::sim2::plugin::Plugin;
  using StateMachine = eufs::sim2::state_machine::StateMachine;

 public:
  /**
   * @brief Construct new SimulationBase object
   * @param core Core simulation
   */
  explicit SimulationBase(std::unique_ptr<core::CoreSimulationBase> core) : core_(std::move(core)) {
    state_machine_ = std::make_unique<StateMachine>();
  }

  /**
   * @brief Run one iteration of the simulation
   * @param dt Amount of time to progress in the simulation
   */
  void Step(const Duration &dt);

  /**
   * @brief Register a new plugin to run with the simulation
   * @param plugin Plugin to run
   */
  void RegisterPlugin(std::unique_ptr<plugin::Plugin> plugin);

 public:
  /**
   * @brief Return reference to track
   * @return Reference to track
   */
  type::Track &GetTrack() { return track_; }

  /**
   * @brief Return read only reference to track
   * @return Read only reference to track
   */
  [[nodiscard]] const type::Track &GetTrack() const { return track_; }

  /**
   * @brief Return read only reference to the initial ground truth track
   * @return Read only reference to ground truth track
   */
  [[nodiscard]] const type::Track &GetGroundTruthTrack() const { return ground_truth_track_; }

  /**
   * @brief Return reference to core simulation
   * @return Reference to core simulation
   */
  core::CoreSimulationBase &GetCore() { return *core_; }

  /**
   * @brief Return read only reference to core simulation
   * @return Read only reference to core simulation
   */
  [[nodiscard]] const core::CoreSimulationBase &GetCore() const { return *core_; }

  /**
   * @brief Return reference to state machine
   * @return Reference to state machine
   */
  StateMachine &GetStateMachine() const { return *state_machine_; }

 public:
  /**
   * @brief Set new control command
   * @param cmd Control commands
   */
  void SetCommand(type::ControlInput cmd) { core_->SetCommand(cmd); }

  /**
   * @brief Set new track and explicit ground truth track in simulation
   * @param track Track used by simulation (potentially modified)
   * @param ground_truth_track Ground truth track (unmodified)
   */
  void SetTrackWithGroundTruth(type::Track track, type::Track ground_truth_track);

  /**
   * @brief Get the current track ID
   */
  uint16_t GetTrackID() const { return track_id_; }

 private:
  // Simulation components
  type::Track track_;
  type::Track ground_truth_track_;
  uint16_t track_id_ = 0;
  std::unique_ptr<core::CoreSimulationBase> core_;
  std::unique_ptr<StateMachine> state_machine_;

  // Plugins
  std::vector<std::unique_ptr<Plugin>> plugins_;
};
}  // namespace eufs::sim2

#endif  // INCLUDE_EUFS_SIM2_SIMULATION_HPP_
