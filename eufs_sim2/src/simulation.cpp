#include "eufs_sim2/simulation.hpp"

#include "eufs_sim2/plugin/base.hpp"

namespace eufs::sim2 {

void SimulationBase::Step(const Duration &dt) {
  // Loop over pre-plugins
  for (auto &plugin : plugins_) {
    plugin->PreUpdate(*this);
  }

  // Update simulation
  core_->Step(dt);

  // Loop over post-plugins
  for (auto &plugin : plugins_) {
    plugin->PostUpdate(*this);
  }
}

void SimulationBase::RegisterPlugin(std::unique_ptr<plugin::Plugin> plugin) {
  plugins_.push_back(std::move(plugin));
}


void SimulationBase::SetTrackWithGroundTruth(type::Track track, type::Track ground_truth_track) {
  track_ = std::move(track);
  ground_truth_track_ = std::move(ground_truth_track);
  track_id_ += 1;
}

}  // namespace eufs::sim2
