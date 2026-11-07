#ifndef INCLUDE_EUFS_SIM2_TYPE_STATE_HPP_
#define INCLUDE_EUFS_SIM2_TYPE_STATE_HPP_

#include <state_lib/state.hpp>

namespace eufs::sim2::type {
struct VehicleStateMember {
  // Define the name of each vector position index
  inline static constexpr state::StateVars<0> _x = {};
  inline static constexpr state::StateVars<1> _y = {};
  inline static constexpr state::StateVars<2> _z = {};
  inline static constexpr state::StateVars<3> _roll = {};
  inline static constexpr state::StateVars<4> _pitch = {};
  inline static constexpr state::StateVars<5> _yaw = {};
  inline static constexpr state::StateVars<6> _v_x = {};
  inline static constexpr state::StateVars<7> _v_y = {};
  inline static constexpr state::StateVars<8> _v_z = {};
  inline static constexpr state::StateVars<9> _v_roll = {};
  inline static constexpr state::StateVars<10> _v_pitch = {};
  inline static constexpr state::StateVars<11> _v_yaw = {};
  inline static constexpr state::StateVars<12> _a_x = {};
  inline static constexpr state::StateVars<13> _a_y = {};
  inline static constexpr state::StateVars<14> _a_z = {};
  inline static constexpr state::StateVars<15> _a_roll = {};
  inline static constexpr state::StateVars<16> _a_pitch = {};
  inline static constexpr state::StateVars<17> _a_yaw = {};
  inline static constexpr state::StateVars<18> _torque = {};
  inline static constexpr state::StateVars<19> _steering = {};

  // Define commonly used groupings of elements
  inline static constexpr auto _2d_position = (VehicleStateMember::_x, VehicleStateMember::_y);
  inline static constexpr auto _position = (_2d_position, VehicleStateMember::_z);
  inline static constexpr auto _orientation =
      (VehicleStateMember::_roll, VehicleStateMember::_pitch, VehicleStateMember::_yaw);

  inline static constexpr auto _2d_linear_velocity =
      (VehicleStateMember::_v_x, VehicleStateMember::_v_y);
  inline static constexpr auto _linear_velocity = (_2d_linear_velocity, VehicleStateMember::_v_z);
  inline static constexpr auto _angular_velocity =
      (VehicleStateMember::_v_roll, VehicleStateMember::_v_pitch, VehicleStateMember::_v_yaw);

  inline static constexpr auto _2d_linear_acceleration =
      (VehicleStateMember::_a_x, VehicleStateMember::_a_y);
  inline static constexpr auto _linear_acceleration =
      (_2d_linear_acceleration, VehicleStateMember::_a_z);
  inline static constexpr auto _angular_acceleration =
      (VehicleStateMember::_a_roll, VehicleStateMember::_a_pitch, VehicleStateMember::_a_yaw);

  inline static constexpr auto _state_2d =
      (_2d_position, VehicleStateMember::_yaw, _2d_linear_velocity, VehicleStateMember::_v_yaw,
       _2d_linear_acceleration, VehicleStateMember::_a_yaw);
};

inline constexpr auto _state_vars =
    (VehicleStateMember::_x & VehicleStateMember::_y & VehicleStateMember::_z &
     VehicleStateMember::_yaw & VehicleStateMember::_roll & VehicleStateMember::_pitch &
     VehicleStateMember::_v_x & VehicleStateMember::_v_y & VehicleStateMember::_v_z &
     VehicleStateMember::_v_yaw & VehicleStateMember::_v_roll & VehicleStateMember::_v_pitch &
     VehicleStateMember::_a_x & VehicleStateMember::_a_y & VehicleStateMember::_a_z &
     VehicleStateMember::_a_yaw & VehicleStateMember::_a_roll & VehicleStateMember::_a_pitch &
     VehicleStateMember::_torque & VehicleStateMember::_steering);

using VehicleState = state::StateVariable<_state_vars, double>;

}  // namespace eufs::sim2::type

#endif  // INCLUDE_EUFS_SIM2_TYPE_STATE_HPP_
