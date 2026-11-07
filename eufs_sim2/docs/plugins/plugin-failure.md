# Plugin Failure 

Base plugin now also implements a failure service which gives the functionlity to fail plugin/sensor publishing whenever we want. It works by implementing the main function that creates a failure service called `CreateSensorFailureService` which then works by requesting a callback to the `TriggerSensorFailureService`. This function has a timer that gets the failure duration as a parameter which is set to 5 seconds by default. 

The `CreateSensorFailure` service takes a hashmap and creates a separate service for each of the plugins. This was needed because the initial sensor failure service that was meant just for the plugins was later extended to the cone fusion sensors : camera and lidar. The `TriggerSensorFailure` service implements a timer that automatically gets cancelled once the duration is over. Failure flags were also converted to a hashmap for the same reason : we wanted to be able to access the individual failure states. 

The Failure Service is then created for each of the plugins individually in `sim_node.cpp` where all of the plugins are getting initialised. This is a simple call to the CreateSensorFailureService(name) in  sim node right after their initialisation. 

ConeFusion is a special case. Creating failure service for cone fusion does not automatically create a failure service for the individual sensors (camera and lidar). This function was called within `InitSensor()` of the `cone_fusion` plugin. To trigger the sensor failure for the cone fusion sensors we put an if conditional within `ThrottedPostUpdate` of `cone_fusion`.


## Failing a plugin 

To set the failure duration : 

``` ros2 param set /eufs_sim2 <failure_parameter_name> failure_duration```

To call the failure service :

```ros2 service call /sensor_failure/<plugin_name> std_srvs/srv/Trigger```









