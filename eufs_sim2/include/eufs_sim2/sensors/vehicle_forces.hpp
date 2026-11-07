#ifndef INCLUDE_EUFS_SIM2_SENSORS_VEHICLE_FORCES_HPP_
#define INCLUDE_EUFS_SIM2_SENSORS_VEHICLE_FORCES_HPP_

#include <state_lib/state.hpp>

namespace eufs::sim2::sensors {

namespace VehicleForcesMember {
// Define the name of each vector position index
inline constexpr state::StateVars<0> _fl_la = {};   // Front left lateral
inline constexpr state::StateVars<1> _fl_lo = {};   // Front left longitudinal
inline constexpr state::StateVars<2> _fl_ve = {};   // Front left vertical

inline constexpr state::StateVars<3> _fr_la = {};   // Front right lateral
inline constexpr state::StateVars<4> _fr_lo = {};   // Front right longitudinal
inline constexpr state::StateVars<5> _fr_ve = {};   // Front right vertical

inline constexpr state::StateVars<6> _rl_la = {};   // Rear left lateral
inline constexpr state::StateVars<7> _rl_lo = {};   // Rear left longitudinal
inline constexpr state::StateVars<8> _rl_ve = {};   // Rear left vertical

inline constexpr state::StateVars<9> _rr_la = {};   // Rear right lateral
inline constexpr state::StateVars<10> _rr_lo = {};  // Rear right longitudinal
inline constexpr state::StateVars<11> _rr_ve = {};  // Rear right vertical
}  // namespace VehicleForcesMember

using VehicleForces =
    state::StateVariable<VehicleForcesMember::_fl_la &
                         VehicleForcesMember::_fl_lo &
                         VehicleForcesMember::_fl_ve &
                         VehicleForcesMember::_fr_la &
                         VehicleForcesMember::_fr_lo &
                         VehicleForcesMember::_fr_ve &
                         VehicleForcesMember::_rl_la &
                         VehicleForcesMember::_rl_lo &
                         VehicleForcesMember::_rl_ve &
                         VehicleForcesMember::_rr_la &
                         VehicleForcesMember::_rr_lo &
                         VehicleForcesMember::_rr_ve,
                         double>;

}  // namespace eufs::sim2::sensors

#endif  // INCLUDE_EUFS_SIM2_SENSORS_VEHICLE_FORCES_HPP_
