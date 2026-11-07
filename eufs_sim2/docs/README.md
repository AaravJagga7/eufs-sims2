# Welcome!

Welcome to eufs_sim2, the successor of the open-sourced [eufs_sim](https://gitlab.com/eufs/eufs_sim/-/wikis/home). After a few years of amazing experience with eufs_sim, we (Edinburgh University Formula Student (EUFS)) have decided to move away from this simulation mainly due to Classic Gazebo reaching its end of life (EOL). It was also during this moment, that we decided to pursue our goal of developing our own autonomous vehicle racing simulator that is both lightweight and physically accurate. It is our goal to ensure that the simulation is capable of running on any device (including a Raspberry Pi! 👀 ). Unlike the generic physics engine you'd use, The simulator has been developed with the vehicle model at the core of the simulation, and any features are essentially extended from it. Advanced developers are also welcome to develop their own vehicle model with their favourite library, such as [PhysX](https://nvidia-omniverse.github.io/PhysX/physx/5.3.1/), [Project Chronos](https://projectchrono.org/) and many more.

![eufs_sim2](images/autocross-4x.gif)

## Who this simulation is for?
If you are someone who is interested in learning more about Autonomous Systems or maybe learning more about how to utilize Robot Operating Systems (ROS) in your projects, then this simulator is for you!

If you are joining any Formula Student competition, this simulator offers teams to test their Autonomous Driving software stack on preset [rule compliant](https://www.imeche.org/docs/default-source/1-oscar/formula-student/2021/forms/ai/fs-ai-dynamic-events-setup-and-cones-specification.pdf?sfvrsn=2) tracks which are part of the dynamic events at the Formula Student competition. Additionally, to prepare for 'blind' dynamic events such as track drive, users can try their software on randomly generated tracks. Teams can adjust or even add to the existing vehicle models to see their own car complete events. By default, the vehicle has been created following the [FS-AI Technical Information](https://www.imeche.org/events/formula-student/team-information/forms-and-documents). 

In summary, teams can attempt all four FSUK dynamic events, with their own vehicle model, in simulation, long before competition starts!

## Navigating This Guide
- [Getting Started](user-guide/getting-started.md): Contains quick setup instructions to get you up and running with the simulation.
- [Software Architecture](user-guide/sim-architecture.md): For nerds out there who are interested in learning how the simulation is designed.
- [User Interface](user-guide/user-interface.md): How to interact with the simulation via command line interface or configuring your simulation via YAML file.
- [Tutorials](tutorials/basics.md): Shows how you can leverage the simulation to start developing your autonomous stack.
- [Troubleshooting and FAQs](miscellaneous/troubleshooting.md): If you're in trouble, this might be the place for you.
- [Contributors](miscellaneous/CONTRIBUTING.md): Contains list of contributors who have been vital in the development of this project.

Access to full documentation [here](https://eufs.gitlab.io/simulation_group/eufs_sim2/)
