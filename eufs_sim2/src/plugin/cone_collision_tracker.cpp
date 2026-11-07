#include "eufs_sim2/plugin/cone_collision_tracker.hpp"
#include <vector>
#include <string>

namespace eufs::sim2::plugin {

using VehicleState = eufs::sim2::core::VehicleState;

void ConeCollisionTracker::SetupROS(rclcpp::Node::SharedPtr node) {
  node_ = node->create_sub_node(subnode_name_);

  // Declare parameters
  frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 200);
  num_x_boxes = node->declare_parameter<int>(plugin_namespace + "num_x_boxes", 10);
  num_y_boxes = node->declare_parameter<int>(plugin_namespace + "num_y_boxes", 10);
  range_x_dim = node->declare_parameter<float>(plugin_namespace + "range_x_dim", 10);
  range_y_dim = node->declare_parameter<float>(plugin_namespace + "range_y_dim", 10);
  collision_radius = node->declare_parameter<float>(plugin_namespace + "collision_radius", 0.8);

  cones_frame_id = node_->declare_parameter<std::string>(
            plugin_namespace + "cones_frame_id", "map");

  in_range_cones_pub_ = node->create_publisher<ConeMessageType>(
    "/plugin/" + subnode_name_ + "/in_range_cones", 1);
  colliding_cones_pub_ = node->create_publisher<ConeMessageType>(
    "/plugin/" + subnode_name_ + "/colliding_cones", 1);

  RCLCPP_INFO(node_->get_logger(), "Cone Collision Tracker Plugin initialised!");
}

void ConeCollisionTracker::ThrottedPostUpdate(const SimulationBase &sim) {
  // Don't do anything if the track has not been initialised.
  if (sim.GetTrackID() == 0) return;
  // Create a new track grid if the track changes.
  if (track_grid.track_id != sim.GetTrackID()) {
    track_grid = TrackGrid(sim, num_x_boxes, num_y_boxes);
    RCLCPP_INFO(node_->get_logger(), "Track Grid changed!");
  }

  VehicleState::Vector vehicle_state;
  sim.GetCore().GetState(vehicle_state);
  auto car_position = vehicle_state(eufs::sim2::type::VehicleStateMember::_2d_position);

  // Create an x by y box centred on the car
  BoundingBox range;
  range.x_dim = range_x_dim;
  range.y_dim = range_y_dim;
  range.bottom_left = car_position - Eigen::Vector2d(range.x_dim / 2, range.y_dim / 2);

  std::vector<eufs::map::Cone> cones_in_range = GetConesInRange(range);

  std::vector<eufs::map::Cone> colliding_cones = GetCollidingCones(car_position, cones_in_range);

  // Publish cones in range only if anything is listening
  if (in_range_cones_pub_->get_subscription_count() > 0) {
    time::Time curr_time = sim.GetCore().GetTime();
    ConeMessageType in_range_cones_msg = ConvertToMsg(cones_in_range, cones_frame_id, curr_time);

    RCLCPP_DEBUG(node_->get_logger(),
     "Publishing cones in range! Number of cones in range: %ld",
      cones_in_range.size());
    in_range_cones_pub_->publish(in_range_cones_msg);
  }

  // Publish colliding cones only if anything is listening
  if (colliding_cones_pub_->get_subscription_count() > 0) {
    time::Time curr_time = sim.GetCore().GetTime();
    ConeMessageType in_range_cones_msg = ConvertToMsg(colliding_cones, cones_frame_id, curr_time);

    RCLCPP_DEBUG(node_->get_logger(),
     "Publishing colliding cones! Number of cones colliding with car: %ld",
      colliding_cones.size());
    colliding_cones_pub_->publish(in_range_cones_msg);
  }
}

bool ConeCollisionTracker::BoxContainsPoint(BoundingBox box, Eigen::Vector2d point) {
  return (point.x() >= box.bottom_left.x()
        && point.x() <= box.bottom_left.x() + box.x_dim
        && point.y() >= box.bottom_left.y()
        && point.y() <= box.bottom_left.y() + box.y_dim);
}

std::vector<eufs::map::Cone> ConeCollisionTracker::GetConesInRange(BoundingBox range) {
  std::vector<eufs::map::Cone> cones_in_range;
  for (auto track_box : track_grid.grid) {
    for (auto cone : track_box.cones) {
      if (BoxContainsPoint(range, cone.position)) {
        cones_in_range.emplace_back(cone);
      }
    }
  }
  return cones_in_range;
}

std::vector<eufs::map::Cone> ConeCollisionTracker::GetCollidingCones(Eigen::Vector2d car_pos,
                                                  std::vector<eufs::map::Cone> cones_in_range) {
  std::vector<eufs::map::Cone> colliding_cones;
  for (auto cone : cones_in_range) {
    Eigen::Vector2d vec_car_to_cone = cone.position - car_pos;
    if (vec_car_to_cone.norm() < collision_radius) {
      colliding_cones.emplace_back(cone);
    }
  }
  return colliding_cones;
}

ConeMessageType ConeCollisionTracker::ConvertToMsg(
                                            const std::vector<eufs::map::Cone> &cones,
                                            const std::string &frame_id,
                                            const time::Time &time) {
    ConeMessageType msg = eufs::map::ToMsg(cones, 0.0);
    msg.header.stamp = time::TimeToTimeMsg(time);
    msg.header.frame_id = frame_id;
    return msg;
}

}  // namespace eufs::sim2::plugin


