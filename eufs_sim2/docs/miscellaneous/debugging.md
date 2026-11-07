# Debugging

Trying to figure out why your vehicle is misbehaving or maybe your vehicle is apparently floating(?) in the sky because you somehow manage to send z-position to the car (not possible btw) can be annoying to deal with. Fear not, as you can try and debug it with the power of **GDB** or your typical **std::cout** spam!

## Debugging using VS Code

If you are using VS code, then this video is your friend! 

<iframe width="560" height="315" src="https://www.youtube.com/embed/uqqHgYsskJI?si=VDvtV51MXgw3o4j_" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


I'd recommend that you go through the video first, but if you still don't understand what needs to be done to properly debug the simulation with GDB, read below.

Firstly, go to the **Run and Debug** on the left panel and if this is your first time setting it up, there should be a highlighted text with the description:

```
create a launch.json file
```

You should then choose **ROS** -> **ROS: Launch** -> **eufs_sim2** -> **eufs_sim2.launch.py**. This should then generate you a file with the following content at least.

```json
{
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
    "version": "0.2.0",
    "configurations": [

        {
            "name": "ROS: Launch",
            "request": "launch",
            "target": "/home/<user_name>/eufs-master/install/eufs_sim2/share/eufs_sim2/launch/eufs_sim2.launch.py",
            "type": "ros"
        }
    ]
}
```

Once this is done, set breakpoints on the line you want to debug by hovering to the left of the line number. Once you're happy with the breakpoints you have left, now comes the most **important** part that is often missed! You MUST add [debug symbol](https://docs.ros.org/en/humble/How-To-Guides/Getting-Backtraces-in-ROS-2.html#preliminaries) which can be passed as an extra cmake arguments when using `colcon`. Use the following command:

```shell
colcon build --packages-select eufs_sim2 --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

Now, you can simply press the `play` which can be found in the `Run and Debug` tab. This should run `eufs_sim2.launch.py` as previously configured and it will then latched onto the first breakpoint. Make full use of the **Watch** and **Variables** panels to monitor the variables you are interested in! As a side note, you can also hover on top of the variable, and this will show you quick information of the content inside the variable.
