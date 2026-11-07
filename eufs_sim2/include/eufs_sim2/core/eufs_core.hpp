#ifndef INCLUDE_EUFS_SIM2_CORE_EUFS_CORE_HPP_
#define INCLUDE_EUFS_SIM2_CORE_EUFS_CORE_HPP_

#include <vector>

#include "eufs_sim2/core/base.hpp"
#include "vehicle_models/models/dynamic_bicycle/dynamic_bicycle.hpp"
#include "vehicle_models/types/command.hpp"

namespace eufs::sim2::core {

class EufsCore : public CoreSimulationBase {
  using StateMatrix = typename VehicleState::EigenMatrix;

 public:
  explicit EufsCore(eufs::vehicle_models::Param param)
      : model_(param, StateMatrix::Identity()), param_(param) {}

  ~EufsCore() override = default;

  void Step(Duration dt) override;

  void SetCommand(ControlInput cmd) override;

  [[nodiscard]] Time GetTime() const override { return time_; }

  [[nodiscard]] VehicleState::Vector GetState(VehicleState::Vector &vec) const override;

  [[nodiscard]] WheelSpeeds::Vector GetState(WheelSpeeds::Vector &vec) const override;

  [[nodiscard]] VehicleForces GetVehicleForces() const override;

  void Reset() override;

 private:
  eufs::vehicle_models::DynamicBicycle<VehicleState, type::VehicleStateMember> model_;
  eufs::vehicle_models::Command command_{};
  VehicleState state_;
  Time time_{};
  eufs::vehicle_models::Param param_;
};
}  // namespace eufs::sim2::core

#endif  // INCLUDE_EUFS_SIM2_CORE_EUFS_CORE_HPP_
