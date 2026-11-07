# Configuration file

Plugin parameters are easily modified through a configuration file located in `/config/plugin_params.yaml`. A configuration file may look like the following:


```yaml
eufs_sim2:
  ros__parameters:
    frequency: 200.0
    plugin_names:
      - track_changer_plugin
      - control_input
      - state_machine
      - vehicle_state_plugin
      - wheel_speed_plugin
      - gnss_plugin
      - oss_plugin
      - imu_plugin
      - state_publisher
      - gt_transform
      - force_publisher
      - twist_publisher
      - cone_fusion
    plugin:
      cone_fusion:
        frequency: 15.0
        cones_frame_id: "base_footprint"
        map_frame_id: "map"
        publish_map: true
        publish_gt_map: true
        publish_gt_cones: true
        sensor_names:
          - camera
          - lidar
        camera:
          fov_radians: 1.91889
          min_view_distance_meters: 1.0
          max_view_distance_meters: 20.0
          colour: true
          publish_own_cones: true
          cones_topic: "camera/cones"
          enabled: true
        lidar:
          fov_radians: 3.141593
          min_view_distance_meters: 1.0
          max_view_distance_meters: 100.0
          colour: false
          publish_own_cones: true
          cones_topic: "lidar_grid/cones"
          enabled: true
      
      track_changer_plugin:
        remove_cones_num: 0
        color_change_to_orange: 0
        color_change_to_yellow: 0
        color_change_to_blue: 0
        shift_cones_num: 0
        shift_cones_min_radius: 0.0
        shift_cones_max_radius: 0.0

      state_machine:
        enforce: false

      vehicle_state_plugin:
        frequency: 20.0

      state_publisher:
        frequency: 20.0
      
      force_publisher:
        frequency: 20.0
      
      gt_transform:
        frequency: 20.0

      wheel_speed_plugin:
        noise_covariance: [
          0.000422806233327, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.000422806233327, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.000422806233327, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.000422806233327, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0001
        ]

      oss_plugin:
        noise_covariance: [
          1.0, 0.0,
          0.0, 1.0
        ]

      imu_plugin:
        angular_velocity_drift_vector: [0.0, 0.0, 0.0]
        linear_acceleration_drift_vector: [0.0, 0.0, 0.0]
        noise_covariance: [
          1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0
        ]
```

Where:

* `eufs_sim2` is the name of the ROS2 node.

* `ros__parameters` is the section where plugins are configured.

* General parameters, `plugin_names` and `plugin` will be discussed below.



## General parameters

There are two general parameters related to the speed of the simulation. `frequency` indicates how many times a second the vehicle model and plugins are updated; while `sim_step` indicates the time increase per update.

As the simulation is usually intended to run at real time speed, the default `sim_step` is 1/frequency. However, the simulation can be sped up or slowed down by changing the `sim_step` relative to the `frequency`. For example, if we want the simulation to run twice as fast as the real time, that would involve setting a `sim_step` of 2/`frequency`.



## Plugin names

The `plugin_names` section manages plugin initialisation. The prefix of every name is matched in the table below to create a plugin instance with unique parameters and a distinct publishing topic.

<div align="center" markdown>

| Plugin                | Prefix                |
| --------------------- | --------------------- |
| TrackChangerPlugin    | *track_changer_plugin*|
| ControlInputPlugin    | *control_input*       |
| StateMachinePlugin    | *state_machine*       |
| VehicleStatePlugin    | *vehicle_state_plugin*|
| WheelSpeedPlugin      | *wheel_speed_plugin*  |
| GNSSPlugin            | *gnss_plugin*         |
| OSSPlugin             | *oss_plugin*          |
| IMUPlugin             | *imu_plugin*          |
| StatePublisherPlugin  | *state_publisher*     |
| GTTransform           | *gt_transform*        |
| ForcePublisher        | *force_publisher*     |
| TwistPublisher        | *twist_publisher*     |
| ConeFusion            | *cone_fusion*         |

</div>

## Plugin parameters

The `plugin` field enables customization by specifying desired parameters under each plugin name. The available parameters can be obtained by executing `ros2 params list` while eufs_sim2 is running. Alternatively, you can also refer to the table below, which lists all the parameters in the example above.

***All plugins have a `frequency` parameter with a default of 200. To avoid bloat, we have ommited it from the table.***

| Type               | Parameters                         | Default value                                                                                                                                                                                                | Description                                                             |
| ------------------ | ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------- |
| ***Any***          | *frequency*                        | 200                                                                                                                                                                                                          | Frequency that a plugin runs with                                       |
| ConeFusion         | *cones_frame_id*                   | "base_footprint"                                                                                                                                                                                             | What frame the cone topics are published in                             |
|                    | *map_frame_id*                     | "map"                                                                                                                                                                                                        | What frame the map cones are published in                               |
|                    | *publish_map*                      | true                                                                                                                                                                                                         | Whether the map is published or not                                     |
|                    | *publish_gt_map*                   | true                                                                                                                                                                                                         | Whether the unmodified ground truth map is published                    |
|                    | *publish_gt_cones*                 | true                                                                                                                                                                                                         | Whether to publish the ground truth cones                               |
|                    | *sensor_names*                     | - camera <br>- lidar                                                                                                                                                                                         | The names of the sensors you would like to use                          |
| ConeFusion Sensor  | *fov_radians*                      | 3.141593                                                                                                                                                                                                     | The default Field of View (fov) of this sensor                          |
|                    | *min_view_distance_meters*         | 1.0                                                                                                                                                                                                          | The minimum distance this sensor can see cones at                       |
|                    | *max_view_distance_meters*         | 20.0                                                                                                                                                                                                         | The maximum distance at which this sensor can see cones                 |
|                    | *colour*                           | true                                                                                                                                                                                                         | Whether this sensor can detect the colour of the cones                  |
|                    | *publish_own_cones*                | false                                                                                                                                                                                                        | Whether this sensor publishes the cones ***it*** sees to it's own topic |
|                    | *cones_topic*                      | ""                                                                                                                                                                                                           | The topic name to publish only the cones ***it*** sees to               |
|                    | *enabled*                          | true                                                                                                                                                                                                         | Whether this sensor is currently able to detect cones or not            |
| TrackChangerPlugin | *remove_cones_num*                 | 0                                                                                                                                                                                                            | The number of cones to remove from the track                            |
|                    | *color_change_to_orange*           | 0                                                                                                                                                                                                            | The number of cones to change to orange                                 |
|                    | *color_change_to_yellow*           | 0                                                                                                                                                                                                            | The number of cones to change to yellow                                 |
|                    | *color_change_to_blue*             | 0                                                                                                                                                                                                            | The number of cones to change to blue                                   |
|                    | *shift_cones_num*                  | 0                                                                                                                                                                                                            | The number of cones to displace                                         |
|                    | *shift_cones_min_radius*           | 0.0                                                                                                                                                                                                          | The minimum radius cones can be shifted by                              |
|                    | *shift_cones_max_radius*           | 0.0                                                                                                                                                                                                          | The maximum radius cones can be shifted by                              |
| StateMachinePlugin | *enforce*                          | false                                                                                                                                                                                                        | Whether to enforce the rules of the DDT state machine                   |
| WheelSpeedPlugin   | *noise_covariance*                 | [<br>0.000422806233327, 0.0, 0.0, 0.0, 0.0,<br>0.0, 0.000422806233327, 0.0, 0.0, 0.0,<br>0.0, 0.0, 0.000422806233327, 0.0, 0.0,<br>0.0, 0.0, 0.0, 0.000422806233327, 0.0,<br>0.0, 0.0, 0.0, 0.0, 0.0001<br>] | The 5x5 noise covariance matrix for the wheel speeds                    |
| OSSPlugin          | *noise_covariance*                 | [<br>1.0, 0.0,<br>0.0, 1.0<br>]                                                                                                                                                                              | The 2x2 noise matrix for the oss                                        |
| IMUPlugin          | *angular_velocity_drift_vector*    | [0.0, 0.0, 0.0]                                                                                                                                                                                              | The angular velocity drift vector for this imu                          |
|                    | *linear_acceleration_drift_vector* | [0.0, 0.0, 0.0]                                                                                                                                                                                              | The linear acceleration drift vector for this imu                       |
|                    | *noise_covariance*                 | [<br>1.0, 0.0, 0.0, 0.0, 0.0, 0.0,<br>0.0, 1.0, 0.0, 0.0, 0.0, 0.0,<br>0.0, 0.0, 1.0, 0.0, 0.0, 0.0,<br>0.0, 0.0, 0.0, 1.0, 0.0, 0.0,<br>0.0, 0.0, 0.0, 0.0, 1.0, 0.0,<br>0.0, 0.0, 0.0, 0.0, 0.0, 1.0<br>]  | The 6x6 covariance matrix for the imu sensor                            |


<div class="custom-note note">
    <h2> Note </h2>
    <p>Default value is used by the simulation when none is specified by the configuration file.</p>
</div>
# Remappings

Remappings allow to change the ros2 topic interface with the eufs_sim2 node. This means that while certain topics are used to publish and subscribe to information internally, these can be changed for communication outside of the eufs_sim2 node. For example, the map is internally published inside eufs_sim2 to the `"/plugin/cone_fusion/map"` topic, while any other node simply needs to listen to the `"/map"` topic. This is an excellent tool if the default eufs_sim2 interface isn't suitable, and specific topics are needed to interact with other applications. Remappings can be modified in `/launch/eufs_sim2.launch.py`.

