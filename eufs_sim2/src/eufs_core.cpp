#include "eufs_sim2/core/eufs_core.hpp"

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <cmath>

namespace eufs::sim2::core {

void EufsCore::Step(Duration dt) {
  state_ = model_.Update(state_, command_, dt);
  time_ += dt;
}

void EufsCore::SetCommand(eufs::sim2::core::ControlInput cmd) {
  command_.acceleration = cmd.acceleration;
  command_.steering_angle = cmd.steering_angle;
}

VehicleState::Vector EufsCore::GetState(VehicleState::Vector &core_state) const {
  core_state[type::VehicleStateMember::_state_2d] =
      state_.state(type::VehicleStateMember::_state_2d);

  return core_state;
}

WheelSpeeds::Vector EufsCore::GetState(WheelSpeeds::Vector &ws) const {
  double tyre_circumference = param_.tyre.radius * 2 * M_PI;

  const double rear_speed = state_.state(type::VehicleStateMember::_v_x) / tyre_circumference;
  const double front_speed = state_.state(type::VehicleStateMember::_v_x) /
                             cos(command_.steering_angle) / tyre_circumference;

  ws[sensors::_wheel_speeds] = {rear_speed, rear_speed, front_speed, front_speed};

  ws(sensors::WheelSpeedsMember::_steering) =
      state_.state(type::VehicleStateMember::_steering);
  return ws;
}

VehicleForces EufsCore::GetVehicleForces() const {
  VehicleForces vf;

  double shared_longitudinal =
      model_.base_model_.GetLongitudinalForce(state_.state, command_) / 2.0;
  double front_lateral =
      model_.base_model_.GetFrontTyreLateralForce(state_.state, command_) / 2.0;
  double rear_lateral =
      model_.base_model_.GetRearTyreLateralForce(state_.state, command_) / 2.0;

  vf.state[sensors::VehicleForcesMember::_fl_la] = front_lateral;
  vf.state[sensors::VehicleForcesMember::_fl_lo] = shared_longitudinal;
  vf.state[sensors::VehicleForcesMember::_fl_ve] = 0;

  vf.state[sensors::VehicleForcesMember::_fr_la] = front_lateral;
  vf.state[sensors::VehicleForcesMember::_fr_lo] = shared_longitudinal;
  vf.state[sensors::VehicleForcesMember::_fr_ve] = 0;

  vf.state[sensors::VehicleForcesMember::_rl_la] = rear_lateral;
  vf.state[sensors::VehicleForcesMember::_rl_lo] = shared_longitudinal;
  vf.state[sensors::VehicleForcesMember::_rl_ve] = 0;

  vf.state[sensors::VehicleForcesMember::_rr_la] = rear_lateral;
  vf.state[sensors::VehicleForcesMember::_rr_lo] = shared_longitudinal;
  vf.state[sensors::VehicleForcesMember::_rr_ve] = 0;

  return vf;
}

void EufsCore::Reset() {
  state_ = VehicleState();
  command_ = eufs::vehicle_models::Command();
}
}  // namespace eufs::sim2::core
