#ifndef INCLUDE_EUFS_SIM2_PLUGIN_CONE_COLLISION_TRACKER_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_CONE_COLLISION_TRACKER_HPP_

#include <utility>
#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <eufs_msgs/msg/cone_with_color_probability_array.hpp>

#include "map_lib/type/cone.hpp"

#include "eufs_sim2/core/base.hpp"
#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/type/state.hpp"
#include "eufs_sim2/simulation.hpp"

#define EPSILON 0.00001

namespace eufs::sim2::plugin {

using ConeMessageType = eufs_msgs::msg::ConeWithColorProbabilityArray;

struct BoundingBox {
  double x_dim;
  double y_dim;
  Eigen::Vector2d bottom_left;
};

struct TrackGridBox {
  BoundingBox box;
  std::vector<eufs::sim2::type::Cone> cones;
};

class TrackGrid {
 private:
    double x_dim;
    double y_dim;
    int num_x_boxes;
    int num_y_boxes;
    Eigen::Vector2d bottom_left;

    /**
     * @brief Find the dimensions of the grid by finding the maximum
     * and minimum cone coordinate values.
     * @param cones: 2D positions of the cones on the track
     */
    void GetGridDimensions(std::vector<eufs::sim2::type::Cone> cones) {
      double min_x = 0;
      double max_x = 0;
      double min_y = 0;
      double max_y = 0;

      for (auto &cone : cones) {
        Eigen::Vector2d pos = cone.position;

        if (pos.x() < min_x) min_x = pos.x();
        else if (pos.x() > max_x) max_x = pos.x();

        if (pos.y() < min_y) min_y = pos.y();
        else if (pos.y() > max_y) max_y = pos.y();
      }

      bottom_left = Eigen::Vector2d(min_x, min_y);
      x_dim = max_x - min_x + EPSILON;
      y_dim = max_y - min_y + EPSILON;
    }

 public:
    std::vector<TrackGridBox> grid;
    int track_id;

    TrackGrid() {}

    /**
     * @brief Create a grid representation of the track.
     * @param sim Reference to base simulation
     * @param num_x_boxes Number of boxes in the x direction
     * @param num_y_boxes Number of boxes in the y direction
     */
    TrackGrid(const SimulationBase &sim, int num_x_boxes, int num_y_boxes) {
      track_id = sim.GetTrackID();
      std::vector<eufs::sim2::type::Cone> cones = sim.GetTrack().ToCones();
      GetGridDimensions(cones);

      double box_x_dim = x_dim / num_x_boxes;
      double box_y_dim = y_dim / num_y_boxes;

      for (int i = 0; i < num_y_boxes; i++) {
        for (int j = 0; j < num_x_boxes; j++) {
          BoundingBox box;
          box.x_dim = box_x_dim + EPSILON;
          box.y_dim = box_y_dim + EPSILON;
          box.bottom_left = Eigen::Vector2d(this->bottom_left.x() + box_x_dim * j,
                                        this->bottom_left.y() + box_y_dim * i);
          TrackGridBox tg_box;
          tg_box.box = box;
          grid.emplace_back(tg_box);
        }
      }

      for (auto &cone : cones) {
        Eigen::Vector2d pos = cone.position;

        int i = (pos.x() - this->bottom_left.x()) / box_x_dim;
        int j = (pos.y() - this->bottom_left.y()) / box_y_dim;

        grid[i + num_x_boxes * j].cones.emplace_back(cone);
      }
    }
};

class ConeCollisionTracker : public Plugin {
 public:
  // Inherit the constructor
  using Plugin::Plugin;

  /**
   * @brief Perform action at a set frequency (optional)
   * @param sim Simulation
   */
  void ThrottedPostUpdate(const SimulationBase &sim) override;


  /**
   * @brief Initialise the ROS interface of the plugin
   * @param node Node handler for the ROS interface
   */
  void SetupROS(rclcpp::Node::SharedPtr node) override;

 private:
  int num_x_boxes;
  int num_y_boxes;
  float range_x_dim;
  float range_y_dim;
  float collision_radius;
  TrackGrid track_grid;

  /** 
   * @brief Does the bounding box contain this point?
   * @param box bounding box
   * @param point point in question
   */
  bool BoxContainsPoint(BoundingBox box, Eigen::Vector2d point);

  /**
   * @brief Get all the cones that lie within a bounding box surrounding the car.
   * @param range the bounding box
   * @return
   */
  std::vector<eufs::map::Cone> GetConesInRange(BoundingBox range);

  /** 
   * @brief Get all the cones that the car collides with.
   * @param car_pos current car position
   * @param cones_in_range 2D positions of cones in bounding box near car
   * @return
   */ 
  std::vector<eufs::map::Cone> GetCollidingCones(Eigen::Vector2d car_pos,
                                                 std::vector<eufs::map::Cone> cones_in_range);

  /**
   * @brief Convert array of cones to ConeMessageType message
   * @param cones 2D positions of cones
   * @return
   */
  ConeMessageType ConvertToMsg(const std::vector<eufs::map::Cone> &cones,
                               const std::string &frame_id,
                               const time::Time &time);

  rclcpp::Publisher<ConeMessageType>::SharedPtr in_range_cones_pub_;
  rclcpp::Publisher<ConeMessageType>::SharedPtr colliding_cones_pub_;
  std::string cones_frame_id;
};

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_CONE_COLLISION_TRACKER_HPP_
