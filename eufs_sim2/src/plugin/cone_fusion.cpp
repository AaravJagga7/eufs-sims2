#include "eufs_sim2/plugin/cone_fusion.hpp"

namespace eufs::sim2::plugin {

using VehicleState = eufs::sim2::type::VehicleState;

void ConeFusion::SetupROS(rclcpp::Node::SharedPtr node) {
    node_ = node->create_sub_node(subnode_name_);

    // Declare params
    frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 200);

    cones_frame_id = node_->declare_parameter<std::string>(
            plugin_namespace + "cones_frame_id", "base_footprint");
    map_frame_id = node_->declare_parameter<std::string>(
            plugin_namespace + "map_frame_id", "map");

    publish_map = node_->declare_parameter<bool>(plugin_namespace + "publish_map", true);
    publish_gt_map = node_->declare_parameter<bool>(plugin_namespace + "publish_gt_map", true);
    publish_gt_cones = node_->declare_parameter<bool>(plugin_namespace + "publish_gt_cones", true);

    // Create sensors
    node_->declare_parameter<std::vector<std::string>>(plugin_namespace + "sensor_names");
    rclcpp::Parameter sensor_names_param = node_->get_parameter(plugin_namespace + "sensor_names");
    std::vector<std::string> sensor_names =
        sensor_names_param.get_value<std::vector<std::string>>();

    for (const std::string &name : sensor_names) {
        InitSensor(plugin_namespace + name);
    }

    ground_truth_cones_pub_ = node_->create_publisher<ConeType>(
        "/plugin/" + subnode_name_ + "/gt_cones", 1);

    perception_cones_pub_ = node_->create_publisher<ConeType>(
        "/plugin/" + subnode_name_ + "/cones", 1);

    map_pub_ = node_->create_publisher<ConeType>(
        "/plugin/" + subnode_name_ + "/map", rclcpp::QoS(1).transient_local());

    ground_truth_map_pub_ = node_->create_publisher<ConeType>(
        "/plugin/" + subnode_name_ + "/ground_truth/map", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(node_->get_logger(), "Cone Fusion Plugin Initialised!");
}

void ConeFusion::InitSensor(const std::string &name) {
    // One by one grab the param values from the yaml
    Sensor new_sensor;

    std::string sensor_name = name.substr(name.find_last_of('.') + 1);
    new_sensor.name = sensor_name;

    new_sensor.fov = node_->declare_parameter<double>(name + ".fov_radians", 3.141593);

    new_sensor.min_view_distance =
        node_->declare_parameter<double>(name + ".min_view_distance_meters", 1.0);

    new_sensor.max_view_distance =
        node_->declare_parameter<double>(name + ".max_view_distance_meters", 20.0);

    new_sensor.colour = node_->declare_parameter<bool>(name + ".colour", true);

    new_sensor.publish_own_cones = node_->
        declare_parameter<bool>(name + ".publish_own_cones", false);

    new_sensor.cones_topic = node_->
        declare_parameter<std::string>(name + ".cones_topic", sensor_name);

    if (new_sensor.publish_own_cones) {
        new_sensor.cones_pub = node_->create_publisher<ConeType>(
        "/plugin/" + subnode_name_ + "/" + new_sensor.cones_topic, 5);
    }

    new_sensor.enabled = node_->declare_parameter<bool>(name + ".enabled", true);

    CreateSensorFailureService(sensor_name);

    sensors_.emplace_back(new_sensor);
}

bool ConeFusion::inRange(eufs::map::Cone cone, double minRange, double maxRange) {
    auto dist = cone.position.norm();
    return minRange <= dist && dist <= maxRange;
}

bool ConeFusion::inView(eufs::map::Cone cone, double fov) {
    float angle = atan2(cone.position(1), cone.position(0));
    return abs(angle) <= (fov / 2);
}

std::pair<eufs::map::Color, bool> ConeFusion::fovCone(const Sensor &sensor,
    const eufs::map::Cone& cone) {
    // Initially check if enabled to avoid wasted computation
    if (!sensor.enabled) return std::pair(eufs::map::Color::UNKNOWN, false);

    bool visible = inRange(cone, sensor.min_view_distance, sensor.max_view_distance)
        && inView(cone, sensor.fov);

    eufs::map::Color cone_color = eufs::map::Color::UNKNOWN;
    if (sensor.colour) {
        cone_color = cone.GetColor();
    }

    return std::make_pair(cone_color, visible);
}

ConeType ConeFusion::ConvertToMsg(
    const std::vector<eufs::map::Cone> &cones,
    const std::string &frame_id,
    const time::Time &time) {
  ConeType msg = eufs::map::ToMsg(cones, 0.0);
  msg.header.stamp = time::TimeToTimeMsg(time);
  msg.header.frame_id = frame_id;
  return msg;
}

std::vector<eufs::map::Cone> ConeFusion::FuseCones(const SimulationBase &sim) {
    std::vector<eufs::map::Cone> cones = sim.GetTrack().ToCones();
    time::Time cur_time = sim.GetCore().GetTime();

    VehicleState::Vector car_state;
    sim.GetCore().GetState(car_state);
    double yaw = car_state(eufs::sim2::type::VehicleStateMember::_yaw);
    auto car_xy = car_state(eufs::sim2::type::VehicleStateMember::_2d_position);

    std::vector<eufs::map::Cone> translated_cones;

    for (const auto& cone : cones) {
        Eigen::Vector2d cone_pos = cone.position;
        eufs::map::Color cone_color = cone.GetColor();

        if (this->cones_frame_id == "base_footprint") {
            // Translate all cones wrt to base footprint
            Eigen::Vector2d translated_cone_pos = cone_pos -= car_xy;

            // Rotate the points using the yaw of the car (2D Rotation Matrix about z-axis)
            translated_cone_pos = Eigen::Rotation2D(-yaw) * translated_cone_pos;

            // Check if cone is within range of perception sensors
            eufs::map::Cone translated_cone = eufs::map::Cone(translated_cone_pos, cone_color);

            eufs::map::Color perception_cone_colour = eufs::map::Color::UNKNOWN;
            bool cone_visible = false;
            for (Sensor& sensor : sensors_) {
                if (!sensor.enabled) continue;

                eufs::map::Color sensed_colour;
                bool visible_by_sensor;
                std::tie(sensed_colour, visible_by_sensor) = fovCone(sensor, translated_cone);

                if (!visible_by_sensor) continue;
                cone_visible = true;

                if (sensed_colour == eufs::map::Color::UNKNOWN) {
                    sensor.cones.emplace_back(eufs::map::Cone(translated_cone_pos,
                                                          sensed_colour));
                } else {
                    perception_cone_colour = sensed_colour;
                    sensor.cones.emplace_back(translated_cone);
                }
            }

            // Remove cones that cannot be seen by any perception sensor
            if (!cone_visible) continue;

            translated_cones.emplace_back(eufs::map::Cone(translated_cone_pos,
                                                          perception_cone_colour));
        }
    }

    return translated_cones;
}

void ConeFusion::ThrottedPostUpdate(const SimulationBase &sim) {
    std::vector<eufs::map::Cone> map_cones = sim.GetTrack().ToCones();
    std::vector<eufs::map::Cone> ground_truth_map_cones = sim.GetGroundTruthTrack().ToCones();
    time::Time cur_time = sim.GetCore().GetTime();

    // Publish the map if something is listening for it
    if (map_pub_->get_subscription_count() > 0) {
        if (publish_map) {
            ConeType map_msg =
                            ConvertToMsg(map_cones, map_frame_id, cur_time);

            RCLCPP_DEBUG(node_->get_logger(), "Map published!");
            map_pub_->publish(map_msg);
        }
    }

    // Publish the unmodified ground truth map if something is listening for it
    if (ground_truth_map_pub_->get_subscription_count() > 0) {
        if (publish_gt_map) {
            ConeType ground_truth_map_msg =
                            ConvertToMsg(ground_truth_map_cones, map_frame_id, cur_time);

            RCLCPP_DEBUG(node_->get_logger(), "Ground truth map published!");
            ground_truth_map_pub_->publish(ground_truth_map_msg);
        }
    }

    // Only publishes cones if something is listening
    if (ground_truth_cones_pub_->get_subscription_count() > 0) {
        if (publish_gt_cones) {
            std::vector<eufs::map::Cone> translated_cones = FuseCones(sim);
            cones_fused = true;

            ConeType translated_cones_msg = ConvertToMsg(
                translated_cones, cones_frame_id, cur_time);

            RCLCPP_DEBUG(node_->get_logger(), "Publishing Ground truth cones");
            ground_truth_cones_pub_->publish(translated_cones_msg);
        }
    }

    // Only publish individual sensor cones when something is listening
    for (Sensor& sensor : sensors_) {
        if (!sensor.publish_own_cones) {
            sensor.cones.clear();
            continue;
        }
        if (sensor_failure_flags_[sensor.name]) continue;
        if (sensor.cones_pub->get_subscription_count() > 0) {
            if (!cones_fused) {
                FuseCones(sim);
                cones_fused = true;
            }

            ConeType sensor_cones_msg = ConvertToMsg(
                sensor.cones, cones_frame_id, cur_time);

            sensor.cones_pub->publish(sensor_cones_msg);
        }
        sensor.cones.clear();
    }

    cones_fused = false;
}

}  // namespace eufs::sim2::plugin
