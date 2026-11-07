There are two main ways to create a new custom plugin. You can either use an **existing plugin** with your custom parameters or, if you feel more confident, **programm** your own.


## Use an existing plugin

1. Decide which plugin you want to create.
2. Write the name of the new plugin under *plugin_names* in `/config/plugin_params.yaml`. Ensure that your name begins with a [supported prefix](./configuration.md#plugin-names).
3. Specify your desired [parameters](./configuration.md#plugin-parameters) under *plugin:yourPluginName*.
4. Consider whether you need to [remap](./configuration.md#) to a topic of your choice.


## Program a new plugin

### Step 0 - Reading

Understand how the [base plugin](./base.md) works, review the implementation of similar plugins, and decide whether your plugin needs PreUpdate or PostUpdate methods.



### Step 1 - Templates

This step provides templates to help you get started with plugin creation. If you have significant programming experience and feel confident, you can skip this step.


First, create a **header** file in `/include/eufs_sim2/plugin` and name it after your plugin, such as `yourPlugin.hpp`. Then, replace the *YourPluginName* and *YOUR_PLUGIN_NAME* placeholders with your plugin's name.



```cpp
#ifndef INCLUDE_EUFS_SIM2_PLUGIN_YOUR_PLUGIN_NAME_HPP_
#define INCLUDE_EUFS_SIM2_PLUGIN_YOUR_PLUGIN_NAME_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/simulation.hpp"
#include "eufs_sim2/core/base.hpp"

namespace eufs::sim2::plugin {

class YourPluginName : public Plugin {
 public:
  // Inherit the constructor
  using Plugin::Plugin;

  /**
   * @brief Perform action before simulation update step (optional)
   * @param sim Simulation
   */
  // void PreUpdate(SimulationBase &sim) override;

  /**
   * @brief Perform action at a set frequency (optional)
   * @param sim Simulation
   */
  // void ThrottedPostUpdate(const SimulationBase &sim) override;


  /**
   * @brief Initialise the ROS interface of the plugin
   * @param node Node handler for the ROS interface
   */
  void SetupROS(rclcpp::Node::SharedPtr node) override;

 private:
  // Add your private member variables here

  // ROS-related members (publishers, subscribers, services, etc.)
};

}  // namespace eufs::sim2::plugin

#endif  // INCLUDE_EUFS_SIM2_PLUGIN_YOUR_PLUGIN_NAME_HPP_
```


Next, create a **source** file in `/src/plugin` and name it `yourPluginName.cpp`. Again, replace the *yourPluginName* and *your_plugin_name* placeholders with your plugin's name.



```cpp
#include "eufs_sim2/plugin/your_plugin_name.hpp"

#include <functional>
#include <memory>

#include "eufs_sim2/simulation.hpp"

namespace eufs::sim2::plugin {

void YourPluginName::SetupROS(rclcpp::Node::SharedPtr node) {
  node_ = node->create_sub_node(subnode_name_);
  frequency_ = node_->declare_parameter<double>(plugin_namespace + "frequency", 200);

  // Add your ROS setup code here
  // Examples:
  // - Declare ROS2 parameters
  // - Create publishers
  // - Create subscribers
  // - Create services
}

// Uncomment and implement if needed
// void YourPluginName::PreUpdate(SimulationBase &sim) {
//   // Add your pre-update logic here
// }

// Uncomment and implement if needed
// void YourPluginName::ThrottedPostUpdate(const SimulationBase &sim) {
//   // Add your throttled post-update logic here
// }

// Add any additional helper methods or callback functions here

}  // namespace eufs::sim2::plugin

```


### Step 2 - Coding


The template above inherits from the base plugin, which has some virtual methods yet to be implemented. Complete the *SetupROS*, *PreUpdate*, and *PostUpdate* methods in the **source** file, and declare any new variables and helper methods in the **header** one.



#### Help

At this point, you might have some questions. This section aims to resolve them and help with the plugin creation process.

<div class="custom-note tips">
    <h2> Use other plugins as example </h2>
    <p> For instance, if you're unsure how to implement a service, you can have a look on how it is implemented on the State Machine plugin. </p>
</div>

<div class="custom-note tips">
    <h2> A common helper function is ConvertToMsg</h2>
    <p> Use it to create and fill the information of a message which will be published. </p>
</div>

<div class="custom-note tips">
    <h2> Need help retrieving information from eufs_sim2? </h2>

    <p> The <code>simulation</code> object, which is given as a reference in the <strong>PreUpdate</strong> and <strong>ThrottedPostUpdate</strong> methods, can be used to obtain references to the track, core (eufs_core), and state machine. This object, is also able to modify the vehicle's command and the track.
    <br>
    <br>
    Additionally, eufs_core allows you to retrieve time, vehicle state, wheel speeds and vehicle forces. It also allows to set the vehicle's command, and reset the vehicle state. For example, time can be retrieved by using <code>sim.GetCore().GetTime();</code>.
    </p>

  </div>

<div class="custom-note tips">
    <h2> vehicle_model & map_lib </h2>
    <p> If your plugin specializes in either vehicle motion or map-related functionality, you may benefit from examining these packages. </p>
</div>