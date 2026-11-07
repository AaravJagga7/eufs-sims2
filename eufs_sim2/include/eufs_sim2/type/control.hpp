#ifndef INCLUDE_EUFS_SIM2_TYPE_CONTROL_HPP_
#define INCLUDE_EUFS_SIM2_TYPE_CONTROL_HPP_

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

namespace eufs::sim2::type {

struct ControlInput {
  double acceleration;
  double steering_angle;
};

/**
 * @brief Convert AckermannDriveStamped message to ControlInput
 * @param msg AckermannDriveStamped message
 * @return ControlInput
 */
ControlInput ToControlInput(const ackermann_msgs::msg::AckermannDriveStamped &msg);

}  // namespace eufs::sim2::type

#endif  // INCLUDE_EUFS_SIM2_TYPE_CONTROL_HPP_
