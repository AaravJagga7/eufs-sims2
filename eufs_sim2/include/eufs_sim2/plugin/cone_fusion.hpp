#ifndef INCLUDE_EUFS_SIM2_PLUGIN_CONE_FUSION_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_CONE_FUSION_HPP_

#include <utility>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <eufs_msgs/msg/cone_with_color_probability_array.hpp>
#include <eufs_msgs/msg/cone_with_color_probability.hpp>

#include "map_lib/type/cone.hpp"

#include "eufs_sim2/core/base.hpp"
#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/type/state.hpp"
#include "eufs_sim2/simulation.hpp"

namespace eufs::sim2::plugin {

using ConeType = eufs_msgs::msg::ConeWithColorProbabilityArray;

class ConeFusion : public Plugin {
 public:
    // Inherit the constructor
    using Plugin::Plugin;

    /**
     * @brief Perform action at a set frequency
     * @param sim Simulation
     */
    void ThrottedPostUpdate(const SimulationBase &sim) override;

    /**
     * @brief Initialise the ROS interface of the plugin
     * @param node Node handler for the ROS interface
     */
    void SetupROS(rclcpp::Node::SharedPtr node) override;


 private:
    struct Sensor {
        std::string name;
        double fov = 3.141593;
        double min_view_distance = 1.0;
        double max_view_distance = 20.0;
        bool colour = true;

        std::vector<eufs::map::Cone> cones;
        bool publish_own_cones = false;
        std::string cones_topic = "";
        rclcpp::Publisher<ConeType>::SharedPtr cones_pub;

        bool enabled = true;
    };

    /**
     * @brief Sets up a sensor gives its namespace
     * @param name Namespace of sensor in the plugin_params.yaml file
     */
    void InitSensor(const std::string &name);

    /**
     * @brief Wrapper function for perception helper function, bool returns
     * if the cone can or cannot be seen by perception sensor
     * @param cone Cone information
     */
    std::pair<eufs::map::Color, bool> fovCone(const Sensor &sensor, const eufs::map::Cone& cone);

    /**
     * @brief Is the cone in range?
     * @param min minimum range of sensor
     * @param max maximum range of sensor
     * @return
     */
    bool inRange(eufs::map::Cone, double min, double max);

    /**
     * @brief Is the cone within the fov of the sensor?
     * @param fov FoV of sensor
     * @return
     */
    bool inView(eufs::map::Cone, double fov);

    /**
     * @brief convert arrays of cones to ConeArrayWithCovariance message
     * @param cones 2D cones' positions with covariance
    */
    ConeType ConvertToMsg(const std::vector<eufs::map::Cone> &cones,
                        const std::string &frame_id,
                        const time::Time &time);

    /**
     * @brief Iterates through sensors and fuses the results.
     * @param sim Reference to the base simulation
     * @return
    */
    std::vector<eufs::map::Cone> FuseCones(const SimulationBase &sim);

    rclcpp::Publisher<ConeType>::SharedPtr ground_truth_cones_pub_;
    rclcpp::Publisher<ConeType>::SharedPtr perception_cones_pub_;
    rclcpp::Publisher<ConeType>::SharedPtr map_pub_;
    rclcpp::Publisher<ConeType>::SharedPtr ground_truth_map_pub_;

    std::string cones_frame_id;
    std::string map_frame_id;

    bool publish_map;
    bool publish_gt_map;
    bool publish_gt_cones;

    std::vector<Sensor> sensors_;

    bool cones_fused = false;
};

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_CONE_FUSION_HPP_
