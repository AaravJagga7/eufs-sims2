#ifndef INCLUDE_EUFS_SIM2_SENSORS_GPS_HPP_
#define INCLUDE_EUFS_SIM2_SENSORS_GPS_HPP_

#include <state_lib/state.hpp>

namespace eufs::sim2::sensors {

namespace GNSSCoordinatesMember {
// Define the name of each vector position index
inline constexpr state::StateVars<0> _lat = {};
inline constexpr state::StateVars<1> _lng = {};
}  // namespace GNSSCoordinatesMember

using WGSCoordinates =
    state::StateVariable<GNSSCoordinatesMember::_lat & GNSSCoordinatesMember::_lng, double>;

}  // namespace eufs::sim2::sensors

#endif  // INCLUDE_EUFS_SIM2_SENSORS_GPS_HPP_
