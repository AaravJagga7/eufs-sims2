#ifndef INCLUDE_EUFS_SIM2_SENSORS_WHEEL_SPEEDS_HPP_
#define INCLUDE_EUFS_SIM2_SENSORS_WHEEL_SPEEDS_HPP_

#include <state_lib/state.hpp>

namespace eufs::sim2::sensors {

namespace WheelSpeedsMember {
// Define the name of each vector position index
inline constexpr state::StateVars<0> _rl = {};        // Rear left
inline constexpr state::StateVars<1> _rr = {};        // Rear right
inline constexpr state::StateVars<2> _fl = {};        // Front left
inline constexpr state::StateVars<3> _fr = {};        // Front right
inline constexpr state::StateVars<4> _steering = {};  // Steering angle
}  // namespace WheelSpeedsMember

inline constexpr auto _wheel_speeds =
  (WheelSpeedsMember::_rl,
    WheelSpeedsMember::_rr,
    WheelSpeedsMember::_fl,
    WheelSpeedsMember::_fr);

inline constexpr auto _wheel_speed_members =
  (sensors::WheelSpeedsMember::_fl &
    sensors::WheelSpeedsMember::_fr &
    sensors::WheelSpeedsMember::_rl &
    sensors::WheelSpeedsMember::_rr &
    sensors::WheelSpeedsMember::_steering);

using WheelSpeeds = state::StateVariable<_wheel_speed_members, double>;
using WheelSpeedsVector = WheelSpeeds::Vector;

}  // namespace eufs::sim2::sensors

#endif  // INCLUDE_EUFS_SIM2_SENSORS_WHEEL_SPEEDS_HPP_
