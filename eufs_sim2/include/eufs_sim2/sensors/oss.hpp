#ifndef INCLUDE_EUFS_SIM2_SENSORS_OSS_HPP_
#define INCLUDE_EUFS_SIM2_SENSORS_OSS_HPP_

#include <state_lib/state.hpp>

#include "eufs_sim2/type/state.hpp"

namespace eufs::sim2::sensors {

inline constexpr auto _oss_members =
  (type::VehicleStateMember::_v_x & type::VehicleStateMember::_v_y);
using OSSVector = state::StateVariable<_oss_members, double>::Vector;

}  // namespace eufs::sim2::sensors

#endif  // INCLUDE_EUFS_SIM2_SENSORS_OSS_HPP_
