# eufs_sim2

A simulation project aimed at validating the software developed for the Formula Student competition, without the need for physical vehicle testing.

Documentation can be found [here](https://eufs.gitlab.io/simulation_group/eufs_sim2/). If you think anything is missing or could be improved, please let us know.

## What is eufs_sim2?

eufs_sim2 is an open source Formula Student autonomous vehicle simulation. It allows teams to test their AI software on preset [rule compliant](https://www.imeche.org/docs/default-source/1-oscar/formula-student/2021/forms/ai/fs-ai-dynamic-events-setup-and-cones-specification.pdf?sfvrsn=2)
tracks which are part of the dynamic events at the Formula Student UK (FSUK) competition as well as for 'blind' dynamic
events such as [autocross or track drive](https://www.imeche.org/docs/default-source/1-oscar/formula-student/2021/forms/ai/fs-2021-autonomous-rules-v1-3.pdf?sfvrsn=2).


## Requirements

- Ubuntu 22.04 LTS
- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)

_Note_: eufs_sim will probably work with other versions of ROS 2 (e.g Foxy) but they are not actively supported by the maintainers.
