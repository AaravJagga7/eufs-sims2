# Getting Started

In this guide, it is expected that you at least:

- Understand how [Linux CLI](https://ubuntu.com/tutorials/command-line-for-beginners#8-hidden-files) works.
- Setting up your [environment variables](https://www.hostinger.co.uk/tutorials/linux-environment-variables) in Linux.
- Know how [ROS2](https://docs.ros.org/en/humble/Tutorials.html) works

## System requirements
Given below are the minimum requirements to run the simulation:

<div align="center" markdown>

|          Type         |        Minimum Requirement       |
|:---------------------:|:--------------------------------:|
| Operating System (OS) |           Ubuntu 22.04           |
|          RAM          |               4 GB               |
|        Storage        | 250 MB (eufs_sim2 + ROS2 Humble) |

</div>

## Installation Guide

This guide ensures that all required tools and dependencies are properly set up to get you up and running with the simulator!

**Step 1: Install ROS2 Humble**

Our simulator is built on top of ros2 Humble. To get started, you need to install ros2 on your system. Please do note that, other versions of ros2 are not tested, hence, we would not be able to help if anything happens. Pull Requests are much appreciated if you would want to contribute in getting it to work with other versions of ros2.

1. **Visit the ros2 Humble Installation page**: Go to [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) documentation to install the package (please install the development tools!).
2. **Follow the installation guide**: Follow the installation guide for instaling the ROS2 Humble on your machine
3. **Environment setup for ros2**: After installation, ensure to source the ROS2 setup script by adding the following into your `.bashrc` or `.zshrc` file.

```bash
# If you are using bash
source /opt/ros/humble/setup.bash

# If you are using zsh
# source /opt/ros/humble/setup.zsh
```

This command will configure your environment to include ROS2 utilities in your path.

**Step 2: Clone eufs_sim2**

Once ROS2 Humble has been installed, clone [eufs_sim2](https://gitlab.com/eufs/simulation_group/eufs_sim2) repository:

1. **Open the terminal**: Navigate to the directory that you want to install eufs_sim2
2. **Create directory**: Create a master directory for your ros2 packages.
3. **Clone repository**: Use the following commands to clone the repository and it's dependencies to your local machine

```bash
# Feel free to name your own master directory
mkdir -p eufs/eufs-master/src
cd eufs/eufs-master/src

git clone https://gitlab.com/eufs/public/eufs_sim2.git
git clone https://gitlab.com/eufs/eufs_msgs.git
git clone https://gitlab.com/eufs/public/eufs-gmock-matchers.git
git clone https://gitlab.com/eufs/public/eufs-logger.git
git clone https://gitlab.com/eufs/public/state_lib.git
git clone https://gitlab.com/eufs/public/map_lib.git
git clone https://gitlab.com/eufs/public/vehicle_models.git

# Install dependencies
sudo rosdep init
rosdep update && rosdep install --from-paths . --ignore-src -r -y
```

**Step 3: Install eufs_cli**

[eufs_cli](https://gitlab.com/eufs/eufs_cli) is eufs's official command line interface and provides a lot of features for interacting with the simulator (among others).

1. **Clone eufs_cli**: In the `eufs` directory that you have created before, clone eufs_cli.
2. **Follow instructions**: Follow the instructions found in the [README.md](https://gitlab.com/eufs/eufs_cli/-/blob/master/README.md?ref_type=heads) file.
3. **Validate eufs cli**: Ensure that the cli works. A simple test would be to do the following:

```bash
eufs --help
```
You should obtain a list of verbs you can use along with the cli as seen below:

```txt
eufs is an easily extensible command-line tool developed by EUFS-AI

options:
  -h, --help  show this help message and exit

Commands:
  Use `eufs <command> -h` for more detailed help

  <command>   description
    branch    Display the current branch on all submodules
    build     Build packages (wrapper for colcon)
    checkout  Checkout the same branch in all submodules
    clean     Remove build and install directories for a specific package or the whole workspace
    data      Command for accessing AI datasets
    install   Installs ROS, clones eufs-master & installs dependencies
    lint      Performs the same linting check as the CI does, and more importantly, can fix errors for you!
    map       Command for accessing map related commands
    perception
              Utility to help with media files and other perception-related tasks
    pull      Pull changes in all submodules
    reset     Resets all submodules
    sim       Command for accessing EUFS sim
    test      Run tests (wrapper for colcon test and colcon test-result)
    update    Update eufscli
```

Feel free to play around with the cli!

**Step 4: Build the simulator**
Now that we have almost everything we need to get everything working, build eufs_sim2 by using the `eufs` cli using the following commands:

```bash
# Navigate to the root directory
cd ~/eufs/eufs-master

# Build direct dependencies
eufs build eufs_msgs eufs_gmock_matchers eufs_logger state_lib

# Build everything else
eufs build eufs_sim2 vehicle_models map_lib

# Source your ROS2 package
. install/setup.bash

# If you are using zsh
# . install/setup.zsh
```

**Step 5: Run the simulator**
Yay! You're now ready to jump into working with the simulator! Run the following command to run the simulator.

```bash
eufs sim run
```

You should get an output that looks something like this

```txt
Running EUFS Sim...
[INFO] [launch]: All log files can be found below /home/espian/.ros/log/2026-03-14-21-59-07-044214-legion-40858
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [40882]
[INFO] [eufs_sim2_node-2]: process started with pid [40884]
[INFO] [trackdrive.csv"}'-3]: process started with pid [40886]
[robot_state_publisher-1] [INFO] [1773525547.192306128] [robot_state_publisher]: got segment base_footprint
[robot_state_publisher-1] [INFO] [1773525547.192385537] [robot_state_publisher]: got segment chassis
[robot_state_publisher-1] [INFO] [1773525547.192398947] [robot_state_publisher]: got segment gps_base_link
[robot_state_publisher-1] [INFO] [1773525547.192408515] [robot_state_publisher]: got segment imu_base_link
[robot_state_publisher-1] [INFO] [1773525547.192418433] [robot_state_publisher]: got segment left_front_wheel
[robot_state_publisher-1] [INFO] [1773525547.192428001] [robot_state_publisher]: got segment left_rear_wheel
[robot_state_publisher-1] [INFO] [1773525547.192437150] [robot_state_publisher]: got segment left_steering_hinge
[robot_state_publisher-1] [INFO] [1773525547.192446160] [robot_state_publisher]: got segment right_front_wheel
[robot_state_publisher-1] [INFO] [1773525547.192455099] [robot_state_publisher]: got segment right_rear_wheel
[robot_state_publisher-1] [INFO] [1773525547.192464807] [robot_state_publisher]: got segment right_steering_hinge
[robot_state_publisher-1] [INFO] [1773525547.192473817] [robot_state_publisher]: got segment velodyne
[robot_state_publisher-1] [INFO] [1773525547.192482826] [robot_state_publisher]: got segment velodyne_base_link
[robot_state_publisher-1] [INFO] [1773525547.192491906] [robot_state_publisher]: got segment zed_camera_center
[robot_state_publisher-1] [INFO] [1773525547.192501055] [robot_state_publisher]: got segment zed_depth_camera_frame
[robot_state_publisher-1] [INFO] [1773525547.192510972] [robot_state_publisher]: got segment zed_left_camera_frame
[robot_state_publisher-1] [INFO] [1773525547.192520471] [robot_state_publisher]: got segment zed_left_camera_optical_frame
[robot_state_publisher-1] [INFO] [1773525547.192529201] [robot_state_publisher]: got segment zed_right_camera_frame
[robot_state_publisher-1] [INFO] [1773525547.192537722] [robot_state_publisher]: got segment zed_right_camera_optical_frame
[eufs_sim2_node-2] [INFO] [1773525547.194567102] [eufs_sim2]: State machine plugin initialised!
[eufs_sim2_node-2] [INFO] [1773525547.196353155] [eufs_sim2]: Vehicle State Plugin Initialised!
[eufs_sim2_node-2] [INFO] [1773525547.198162466] [eufs_sim2]: wheel_speed_plugin plugin initialised!
[eufs_sim2_node-2] [INFO] [1773525547.199009711] [eufs_sim2]: GNSS plugin initialised!
[eufs_sim2_node-2] [INFO] [1773525547.199860589] [eufs_sim2]: oss_plugin plugin initialised!
[eufs_sim2_node-2] [INFO] [1773525547.200631079] [eufs_sim2]: imu_plugin plugin initialised!
[eufs_sim2_node-2] [INFO] [1773525547.207508358] [eufs_sim2]: Cone Fusion Plugin Initialised!
[eufs_sim2_node-2] [INFO] [1773525547.593131029] [eufs_sim2]: Map changed!
[INFO] [trackdrive.csv"}'-3]: process has finished cleanly [pid 40886]
```

You are also able to access all of the ROS2 nodes and topics published from the simulator via the ROS2 cli

```bash
ros2 topic list
```

You should see a list of topics that looks something like this:

```txt
/camera/cones
/clock
/cmd
/cones
/cones/lenient
/driving_flag
/eufs/joint_states
/imu/data
/lidar_grid/cones
/map
/odom
/parameter_events
/plugin/force_publisher/car_forces
/plugin/gnss_plugin/fix
/plugin/oss_plugin/oss/data
/robot_description
/ros_can/mission_completed
/ros_can/twist
/ros_can/wheel_speeds
/rosout
/sim/ros_can/state
/sim/ros_can/state_str
/tf
/tf_static
```