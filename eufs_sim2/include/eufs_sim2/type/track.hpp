#ifndef INCLUDE_EUFS_SIM2_TYPE_TRACK_HPP_
#define INCLUDE_EUFS_SIM2_TYPE_TRACK_HPP_

#include <map_lib/type/cone.hpp>
#include <map_lib/type/map.hpp>

namespace eufs::sim2::type {

using eufs::map::Cone;
using Track = eufs::map::Map<Cone>;

}  // namespace eufs::sim2::type

#endif  // INCLUDE_EUFS_SIM2_TYPE_TRACK_HPP_
