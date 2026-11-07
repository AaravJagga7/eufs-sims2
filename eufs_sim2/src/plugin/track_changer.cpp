#include <filesystem>
#include <iostream>

#include <unordered_set>
#include <random>
#include <algorithm>

#include "eufs_sim2/plugin/track_changer.hpp"

#include <map_lib/io/reader.hpp>
#include <map_lib/type/cone.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace eufs::sim2::plugin {

// Randomly removes a given number of cones from the track
void RemoveCones(type::Track &track, int remove_cones_num) {
  if (remove_cones_num < 0) {
    throw std::invalid_argument("remove_cones_num cannot be negative");
  }

  int total_cones = track.size();
  if (total_cones == 0 || remove_cones_num == 0) return;

  remove_cones_num = std::min(remove_cones_num, total_cones);

  // Generate unique random indices to remove
  std::vector<int> indices(track.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::shuffle(indices.begin(), indices.end(), std::mt19937 {std::random_device {}()});
  indices.resize(remove_cones_num);
  std::sort(indices.begin(), indices.end(), std::greater<>());

  for (int i : indices) {
    track.Remove(i);
  }
}

// Randomly changes the color of cones in the given track
void ChangeConeColors(type::Track &track, int color_change_to_orange,
                      int color_change_to_yellow, int color_change_to_blue) {
  if (color_change_to_orange < 0 || color_change_to_yellow < 0 || color_change_to_blue < 0) {
    throw std::invalid_argument("Color change values must be non-negative");
  }

  int total_cones = track.size();
  if (total_cones == 0) return;

  int requested_total = color_change_to_orange + color_change_to_yellow + color_change_to_blue;
  if (requested_total == 0) return;

  // Clamp total changes to number of available cones
  int total_to_change = std::min(requested_total, total_cones);

  std::vector<int> indices(total_cones);
  std::iota(indices.begin(), indices.end(), 0);
  std::shuffle(indices.begin(), indices.end(), std::mt19937 {std::random_device {}()});

  int changed = 0;

  // Change color to ORANGE
  for (int i = 0; i < color_change_to_orange && changed < total_to_change; ++i, ++changed) {
    track.at(indices[changed]).SetColor(map::Color::ORANGE);
  }
  // Change color to YELLOW
  for (int i = 0; i < color_change_to_yellow && changed < total_to_change; ++i, ++changed) {
    track.at(indices[changed]).SetColor(map::Color::YELLOW);
  }
  // Change color to BLUE
  for (int i = 0; i < color_change_to_blue && changed < total_to_change; ++i, ++changed) {
    track.at(indices[changed]).SetColor(map::Color::BLUE);
  }
}

// Randomly shifts the position of a given number of cones in the track
void ShiftCones(type::Track &track, int shift_cones_num,
                double shift_cones_min_radius, double shift_cones_max_radius) {
  if (shift_cones_num < 0) {
    throw std::invalid_argument("shift_cones_num cannot be negative");
  }
  if (shift_cones_min_radius < 0.0 || shift_cones_max_radius < 0.0) {
    throw std::invalid_argument("Radius values must be non-negative");
  }
  if (shift_cones_min_radius > shift_cones_max_radius) {
    throw std::invalid_argument("min_radius cannot be greater than max_radius");
  }

  int total_cones = track.size();
  if (total_cones == 0 || shift_cones_num == 0) return;

  // Clamp shift_number to the number of cones available
  int num_to_shift = std::min(shift_cones_num, total_cones);

  // Generate a shuffled list of indices for random selection
  std::vector<int> indices(total_cones);
  std::iota(indices.begin(), indices.end(), 0);
  std::shuffle(indices.begin(), indices.end(), std::mt19937 {std::random_device {}()});

  // Setup random generators for radius and angle
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> radius_dist(shift_cones_min_radius, shift_cones_max_radius);
  std::uniform_real_distribution<> angle_dist(0.0, 2.0 * std::numbers::pi_v<double>);

  // Shift cone positions randomly within the specified range
  for (int i = 0; i < num_to_shift; ++i) {
    const double radius = radius_dist(gen);
    const double angle = angle_dist(gen);
    const double dx = radius * std::cos(angle);
    const double dy = radius * std::sin(angle);

    int idx = indices[i];
    track.at(idx).position += Eigen::Vector2d(dx, dy);
  }
}

void TrackChangerPlugin::PreUpdate(SimulationBase &sim) {
  if (track_changed) {
    sim.SetTrackWithGroundTruth(std::move(track_), std::move(ground_truth_track_));
    RCLCPP_INFO(node_->get_logger(), "Map changed!");
  }

  track_changed = false;
}

void TrackChangerPlugin::SetupROS(rclcpp::Node::SharedPtr node) {
  node_ = node->create_sub_node(subnode_name_);
  frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 200);

  // parameters for RemoveCones
  remove_cones_num_ = node_->declare_parameter<int>(
    plugin_namespace + "remove_cones_num", 0);

  // parameters for ChangeConeColors
  color_change_to_orange_ = node_->declare_parameter<int>(
    plugin_namespace + "color_change_to_orange", 0);
  color_change_to_yellow_ = node_->declare_parameter<int>(
    plugin_namespace + "color_change_to_yellow", 0);
  color_change_to_blue_ = node_->declare_parameter<int>(
    plugin_namespace + "color_change_to_blue", 0);

  // parameters for ShiftCones
  shift_cones_num_ = node_->declare_parameter<int>(
    plugin_namespace + "shift_cones_num", 0);
  shift_cones_min_radius_ = node_->declare_parameter<double>(
    plugin_namespace + "shift_cones_min_radius", 0.0);
  shift_cones_max_radius_ = node_->declare_parameter<double>(
    plugin_namespace + "shift_cones_max_radius", 0.0);

  service_file_ = node_->create_service<eufs_msgs::srv::SetString>(
      "~/file", std::bind(&TrackChangerPlugin::TrackFromFile, this, _1, _2));
  service_msg_ = node_->create_service<eufs_msgs::srv::SetTrack>(
      "~/msg", std::bind(&TrackChangerPlugin::TrackFromMsg, this, _1, _2));
  service_get_map_ = node->create_service<eufs_msgs::srv::GetMap>(
      "~/get_map", std::bind(&TrackChangerPlugin::GetMap, this, _1, _2));
}

void TrackChangerPlugin::GetMap(
    [[maybe_unused]] const std::shared_ptr<eufs_msgs::srv::GetMap::Request> req,
    std::shared_ptr<eufs_msgs::srv::GetMap::Response> res) {

    std::vector<std::string> available_maps;

    // Only check maps that are within the maps folder
    std::string map_path =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("map_lib")) / "maps";

    int csvCount = 0;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(map_path)) {
      if (entry.is_regular_file() && entry.path().extension() == ".csv") {
        ++csvCount;
        available_maps.push_back(entry.path());
      }
      RCLCPP_DEBUG(node_->get_logger(), "[TrackChangerPlugin::GetMap] Current file: %s",
                   entry.path().c_str());
    }

    RCLCPP_DEBUG(node_->get_logger(), "[TrackChangerPlugin::GetMap] Number of csv files found: %i",
                 csvCount);
    res->map_path = available_maps;
}

void TrackChangerPlugin::TrackFromFile(
    const std::shared_ptr<eufs_msgs::srv::SetString::Request> req,
    std::shared_ptr<eufs_msgs::srv::SetString::Response> res) {
    try {
      ground_truth_track_ = type::Track(map::io::LoadCones<type::Cone>(req->data));
      track_ = ground_truth_track_;

      RemoveCones(track_,
                  static_cast<int>(remove_cones_num_));

      ChangeConeColors(track_,
                       static_cast<int>(color_change_to_orange_),
                       static_cast<int>(color_change_to_yellow_),
                       static_cast<int>(color_change_to_blue_));

      ShiftCones(track_,
                 static_cast<int>(shift_cones_num_),
                 shift_cones_min_radius_,
                 shift_cones_max_radius_);

      track_changed = true;
      res->success = true;
    } catch (std::runtime_error & e) {
      std::cerr << "Warning: " << e.what() << std::endl;
    }
}

void TrackChangerPlugin::TrackFromMsg(const
  std::shared_ptr<eufs_msgs::srv::SetTrack::Request> req,
  std::shared_ptr<eufs_msgs::srv::SetTrack::Response> res) {
  ground_truth_track_ = type::Track(map::FromMsg<type::Cone>(req->data));
  track_ = ground_truth_track_;

  RemoveCones(track_,
              static_cast<int>(remove_cones_num_));

  ChangeConeColors(track_,
                  static_cast<int>(color_change_to_orange_),
                  static_cast<int>(color_change_to_yellow_),
                  static_cast<int>(color_change_to_blue_));

  ShiftCones(track_,
            static_cast<int>(shift_cones_num_),
            shift_cones_min_radius_,
            shift_cones_max_radius_);

  track_changed = true;
  res->success = true;
}

}  // namespace eufs::sim2::plugin
