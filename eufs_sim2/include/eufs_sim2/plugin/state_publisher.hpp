#ifndef INCLUDE_EUFS_SIM2_PLUGIN_STATE_PUBLISHER_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_STATE_PUBLISHER_HPP_

#include <unordered_map>

#include <state_lib/state.hpp>
#include <eufs_msgs/msg/can_state.hpp>
#include "std_msgs/msg/string.hpp"
#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/simulation.hpp"

namespace eufs::sim2::plugin {
class StatePublisherPlugin : public Plugin {
 public:
    // Inherit the constructor
    using Plugin::Plugin;

    /**
     * @brief Publish ASState and AMIState as both a CanState and a String
     * @param sim Simulation
    */
    void ThrottedPostUpdate(const SimulationBase &sim) override;

    /**
     * @brief Initialise the ROS interface of the plugin
     * @param node Node handler for the ROS interface
    */
    void SetupROS(rclcpp::Node::SharedPtr node) override;

    std::unordered_map<state_machine::AMIState, int> ami_state_translation = {
        {state_machine::AMIState::NOT_SELECTED,
         eufs_msgs::msg::CanState::AMI_NOT_SELECTED},
        {state_machine::AMIState::ACCELERATION,
         eufs_msgs::msg::CanState::AMI_ACCELERATION},
        {state_machine::AMIState::SKIDPAD,
         eufs_msgs::msg::CanState::AMI_SKIDPAD},
        {state_machine::AMIState::AUTOCROSS,
         eufs_msgs::msg::CanState::AMI_AUTOCROSS},
        {state_machine::AMIState::TRACK_DRIVE,
         eufs_msgs::msg::CanState::AMI_TRACK_DRIVE},
        {state_machine::AMIState::DDT_AUTONOMOUS_DEMO,
         eufs_msgs::msg::CanState::AMI_AUTONOMOUS_DEMO},
        {state_machine::AMIState::ADS_INSPECTION,
         eufs_msgs::msg::CanState::AMI_ADS_INSPECTION},
        {state_machine::AMIState::ADS_EBS_TEST,
         eufs_msgs::msg::CanState::AMI_ADS_EBS},
        {state_machine::AMIState::DDT_INSPECTION_A,
         eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_A},
        {state_machine::AMIState::DDT_INSPECTION_B,
         eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_B},
        {state_machine::AMIState::MANUAL_DRIVING,
         eufs_msgs::msg::CanState::AMI_MANUAL}
    };

    std::unordered_map<state_machine::ASState, int> as_state_translation = {
        {state_machine::ASState::OFF, eufs_msgs::msg::CanState::AS_OFF},
        {state_machine::ASState::READY, eufs_msgs::msg::CanState::AS_READY},
        {state_machine::ASState::DRIVING, eufs_msgs::msg::CanState::AS_DRIVING},
        {state_machine::ASState::EMERGENCY_BRAKE, eufs_msgs::msg::CanState::AS_EMERGENCY_BRAKE},
        {state_machine::ASState::FINISHED, eufs_msgs::msg::CanState::AS_FINISHED},
    };

 private:
    eufs_msgs::msg::CanState ConvertToCanMsg(state_machine::AMIState amiState,
        state_machine::ASState asState);

    std_msgs::msg::String ConvertToCanStr(state_machine::AMIState amiState,
                                          state_machine::ASState asState);

    rclcpp::Publisher<eufs_msgs::msg::CanState>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_str_;
};
}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_STATE_PUBLISHER_HPP_
