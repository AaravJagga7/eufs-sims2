#ifndef INCLUDE_EUFS_SIM2_SENSORS_IMU_HPP_
#define INCLUDE_EUFS_SIM2_SENSORS_IMU_HPP_

#include <state_lib/state.hpp>

namespace eufs::sim2::sensors {

inline constexpr auto _imu_members =
  (type::VehicleStateMember::_v_roll &
    type::VehicleStateMember::_v_pitch &
    type::VehicleStateMember::_v_yaw &
    type::VehicleStateMember::_a_x &
    type::VehicleStateMember::_a_y &
    type::VehicleStateMember::_a_z);
using IMUVector = state::StateVariable<_imu_members, double>::Vector;

}  // namespace eufs::sim2::sensors

#endif  // INCLUDE_EUFS_SIM2_SENSORS_IMU_HPP_
