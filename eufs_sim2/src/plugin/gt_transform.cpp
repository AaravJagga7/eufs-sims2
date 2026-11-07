#include "eufs_sim2/plugin/gt_transform.hpp"

#include <functional>
#include <memory>
#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "eufs_sim2/simulation.hpp"
#include "eufs_sim2/time/time.hpp"

using geometry_msgs::msg::TransformStamped;

namespace eufs::sim2::plugin {


// Post Update
void GTTransform::ThrottedPostUpdate(const SimulationBase &sim) {
    time::Time time = sim.GetCore().GetTime();
    VehicleState::Vector sensor_data;
    sim.GetCore().GetState(sensor_data);
    tf_broadcaster_->sendTransform(ConvertToMsg(sensor_data, time));
}

// Set up ROS
void GTTransform::SetupROS(rclcpp::Node::SharedPtr node) {
    node_ = node->create_sub_node(subnode_name_);
    frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 200);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    vehicle_frame_id = node_->declare_parameter<std::string>(plugin_namespace + "vehicle_frame",
                                                             "base_footprint");
}

//  Convert To Message
geometry_msgs::msg::TransformStamped GTTransform::ConvertToMsg(const type::VehicleState::Vector &vs,
                                                               time::Time time) {
    geometry_msgs::msg::TransformStamped msg;

    msg.header.stamp = time::TimeToTimeMsg(time);
    msg.header.frame_id = "map";
    msg.child_frame_id = vehicle_frame_id;

    msg.transform.translation.x = vs(type::VehicleStateMember::_x);
    msg.transform.translation.y = vs(type::VehicleStateMember::_y);
    msg.transform.translation.z = vs(type::VehicleStateMember::_z);

    tf2::Quaternion q;
    q.setRPY(0,
             0,
             vs(type::VehicleStateMember::_yaw));
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();

    return msg;
}
}  // namespace eufs::sim2::plugin
