#ifndef INCLUDE_EUFS_SIM2_CORE_BASE_HPP_
#define INCLUDE_EUFS_SIM2_CORE_BASE_HPP_

#include <vector>

#include <state_lib/state.hpp>

#include "eufs_sim2/sensors/wheel_speeds.hpp"
#include "eufs_sim2/sensors/vehicle_forces.hpp"
#include "eufs_sim2/time/time.hpp"
#include "eufs_sim2/type/control.hpp"
#include "eufs_sim2/type/state.hpp"

namespace eufs::sim2::core {
using Duration = eufs::sim2::time::Duration;
using VehicleState = type::VehicleState;
using ControlInput = eufs::sim2::type::ControlInput;
using Time = eufs::sim2::time::Time;
using WheelSpeeds = eufs::sim2::sensors::WheelSpeeds;
using VehicleForces = eufs::sim2::sensors::VehicleForces;

class CoreSimulationBase {
 public:
  /**
   * @brief Virtual destructor
   */
  virtual ~CoreSimulationBase() = default;

  /**
   * @brief Step through the simulation.
   * @param dt Amount of time to step by
   */
  virtual void Step(Duration dt) = 0;

  /**
   * @brief Set new control command
   * @param cmd Control commands
   */
  virtual void SetCommand(ControlInput cmd) = 0;

  /**
   * @brief Get current simulation time
   * @return Time
   */
  virtual time::Time GetTime() const = 0;

  /**
   * @brief Return the state of the vehicle
   * @param vec
   * @return VehicleState
   */
  virtual VehicleState::Vector GetState(VehicleState::Vector &vec) const = 0;

  /**
   * @brief Return wheel speed information
   * @param vec
   * @return Wheel speeds
   */
  virtual WheelSpeeds::Vector GetState(WheelSpeeds::Vector &vec) const = 0;

  /**
   * @breif Return various vehicle forces
   * @return VehicleForces
   */
  virtual sensors::VehicleForces GetVehicleForces() const = 0;

  /**
   * @brief Reset the simulation
   */
  virtual void Reset() = 0;
};

}  // namespace eufs::sim2::core

#endif  // INCLUDE_EUFS_SIM2_CORE_BASE_HPP_
