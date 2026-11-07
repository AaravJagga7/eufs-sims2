# Troubleshooting

This is a guide on things that can be tried when experiencing any problems with our simulator.



## Sim crashes

1. Check that all the packages are on the latest `master` branch and built on the install folder, If you are developing, make sure you are on the right branch.
2. Inspect the errors on the terminal.
    * If the error seems to be in eufs_sim2, [debug](./debugging.md) it to figure out where the error is located. You can alternative use temporal print statements.
    * If the error seems to be on another package, inspect it and maybe try rebuilding it.
    * If no error shows up, reset the terminal and try again. If still nothing is shown, then try debugging.


<details>
<summary><b>Packages?</b></summary>
<div style="padding: 10px; border: 1px solid #ccc; border-radius: 5px; background-color: #f9f9f9; margin-top: 5px;">
They allow for code modularisation. The complete list of packages can be found in *package.xml* and *CMakeLists.txt*. You should be aware of: eufs_sim2, eufs_msgs, map_lib, state_lib and vehicle_models.
</div>
</details>
<br>


## Recommended procedures


If a certain error message is displayed, you can try and search it on the internet. On the contrary, if nothing is displayed, you can try the following fixes.



* *Clean ROS packages*. Sometimes after deleting code, the old build files may remain on the install folder. You can clean the entire package by using:

        eufs clean <package_name>
        # For example:
        # eufs clean eufs_sim2


    And then build the package again.

        eufs build <package_name>



* *Ensure that you are on the latest master branch*. This is applicable to eufs_sim2, as well as all its dependencies. 

        eufs checkout simulation master
        eufs checkout infrastructure master


* *Source .bashrc, and/or ROS humble*. This makes the ROS packages and funtionality available to your terminal session.

        source .bashrc
        source /opt/ros/humble

* *Kill any ROS2 nodes*. Sometimes a node can continue working on the background, causing problems when you relaunch it.

        # List running nodes
        ros2 node list

        # Search and kill processes
        ps aux | grep <node_name>
        kill -9 <process_number>


* *Restart the terminal and/or computer*. Do it as a last resource after having tried everything else.



### EUFS CLI

Setting up the CLI might prove difficult. In that case, refer to the [README](https://gitlab.com/eufs/eufs_cli) for the latest information. Some common issues include:

* *command not found: eufs*. Refer to the README.
* *eufs checkout* or *eufs branch* not working. Run ```eufs update```.



