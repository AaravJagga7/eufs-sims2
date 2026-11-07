# Plugins

Plugins are a key component of eufs_sim2, offering a flexible, modular, and standardized approach of implementing additional features. Whether you need to implement car sensors, change the track, or monitor the state machine, plugins have you covered! The following sections will explore the workings of plugins, including their common features, configuration, examples, and a final section that teaches you how to create your own plugin.



# Base plugin

In the grand scheme of things, eufs_sim2 is updated on a step-by-step basis, which involves updating the simulation's [vehicle model](), time, and **plugins**. Lets consider how a step is executed:


```c++
void SimulationBase::Step(const Duration &dt) {
  // Loop over pre-plugins
  for (auto &plugin : plugins_) {
    plugin->PreUpdate(*this);
  }

  // Update simulation
  core_->Step(dt);

  // Loop over post-plugins
  for (auto &plugin : plugins_) {
    plugin->PostUpdate(*this);
  }
}
```

First, plugins that **modify** the simulation are executed via their **PreUpdate** method. Then the core simulation is stepped, updating the vehicle model and internal time. Finally, plugins that **read** from the simulation are executed via the **PostUpdate** method.

<div class="custom-note tips">
  <h2> How to know if your plugin should use PreUpdate or PostUpdate? </h2>
  <p> An inuitive example to know if it should fall into either category is again by considering the following keyword <strong>modify</strong> or <strong>read</strong>. For example, in order to drive the car around, you need to update the vehicle model with the commands (e.g. acceleration), this needs to be updated before the vehicle model step, hence it modifies the simulation state. </p>
</div>

To loop accross plugins as specified above, a certain level of standardization is required. This is achieved through a **base class** that defines common methods and variables, inhereted by all plugins. The code for the **base plugin** is:


```c++
class Plugin {
 public:
   /**
   * @brief Virtual destructor
   */
  virtual ~Plugin() = default;

  /**
   * @brief construct plugin with name
   * @param name subnode name
   */
  explicit Plugin(std::string name) :
    subnode_name_(name), plugin_namespace("plugin." + name + ".") {}

  /**
   * @brief Initialise the ROS interface of the plugin
   */
  virtual void SetupROS(rclcpp::Node::SharedPtr) {}

  /**
   * @brief Perform action before simulation update step
   */
  virtual void PreUpdate(SimulationBase &) {}

  /**
   * @brief Runs at a throttled frequency
   */
  virtual void ThrottedPostUpdate(const SimulationBase &) {}

  /**
   * @brief Runs ThrottedPostUpdate at a set frequency
   */
  void PostUpdate(const SimulationBase &);

 protected:
  std::string subnode_name_;
  rclcpp::Node::SharedPtr node_;
  std::string plugin_namespace;
  double frequency_;
  eufs::sim2::time::Time previous_time_ = time::Time::zero();
};
```

Important aspects to note are:

* *Constructor*: Initializes the plugin with its name and namespace. This is necesary to support multiple instances of the same plugin.
* *SetupROS*: Initializes the required ROS2 subscribers, publishers and services.
* *PreUpdate*: Executes plugin functionality at the **beginning** of step function. This is the method to use if the plugin **modifies** the simulation.
* *PostUpdate*: Calls *ThrottedPostUpdate* at a specified frequency. It is used to limit the rate of publishing.
* *ThrottedPostUpdate*: Executes plugin functionality at the **end** of step function. It is usually used for **publishing** data.
 

<details>
<summary><b>ThrottedPreUpdate?</b></summary>
<div style="padding: 10px; border: 1px solid #ccc; border-radius: 5px; background-color: #f9f9f9; margin-top: 5px;">
This method doesn't exist, as we want to modify the simulation as soon as possible.
</div>
</details>
<br>


