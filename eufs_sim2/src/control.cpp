#include "eufs_sim2/type/control.hpp"

namespace eufs::sim2::type {

ControlInput ToControlInput(const ackermann_msgs::msg::AckermannDriveStamped &msg) {
  ControlInput cmd;
  cmd.acceleration = msg.drive.acceleration;
  cmd.steering_angle = msg.drive.steering_angle;
  return cmd;
}

}  // namespace eufs::sim2::type
