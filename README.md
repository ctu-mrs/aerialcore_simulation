# Aerial-Core simulation

Repository with gazebo models and worlds for Aerial-Core project.

## World configuration

The procedural power line spawning is linked with Gazebo topic `/gazebo/default/pose/info`. Gazebo publishes position and orientation of each UAV part in the simulation allowing to generate the world with respect to the specified unit. The default world file(`/worlds/aerial_core_autospawn.world`) incorporate configurable parameter`<uav_name>` setting the reference unit for world spawning. The default value is set to `uav1` for default CTU-MRS simulation. You can set the value to fit your simulation UAV setup.

## Dynamic model plugin 

### Description
The dynamic model plugin enables simulation of dynamic models in Gazebo worlds. The plugin can be added to any gazebo model by addition of following lines to model definition file (`*.sdf`). 

```xml
  	<plugin name="dynamic_model_plugin" filename="libDynamicModelPlugin.so">
       <update_rate>30</update_rate>
       <trajectory_file>circle_trajectory.txt</trajectory_file>
       <use_segmentation>true</use_segmentation>
       <use_directional_yaw>true</use_directional_yaw>
       <initial_on>true</initial_on>
       <loop_enabled>true</loop_enabled>
     </plugin>
```

The parameters have the following meaning:

* **update_rate**
  * frequency of updates of state of the model
* **trajectory_file**
  * path to the file with desired trajectory of the model
  * can be specified as absolute path, as relative path to location of `aerial_core_simulation` package, or as relative path to folder `trajectories` located in folder of the corresponding gazebo model
* **use_segmentation**
  * specifies whether the provided trajectory should be segmented according to required velocity or used directly point by point with the specified `update_rate` 
* **use_directional_yaw**
  * specifies whether the model should use specified yaw or it should be always oriented in the direction of its motion
* **initial_on**
  * specifies whether the trajectory should be followed immediately after spawning the model or it should be initially inactive and activated later by calling activation service
* **loop_enabled**
  * specifies whether the model should stop at the end of the desired trajectory or it should immediately start following the trajectory from its beginning

### Trajectory specification
The trajectory is defined by the text file containing sequence of tuples `x y z roll pitch yaw velocity`, where each tuple is located on a seperate line e.g.
```
  ...
  9.7 2.4 0.0 0.0 0.0 0.0 0.5
  9.6 2.7 0.0 0.0 0.0 1.0 0.5
  9.5 3.0 0.0 0.0 0.0 2.0 0.5
  9.3 3.4 0.0 0.0 0.0 3.0 0.5
  ...
```

If the argument `use_segmentation` is set to true, the trajectory is segmented according to specified velocity.

### ROS interface 

The motion of the model can be activated and deactivated by calling service 
```
rosservice call /gazebo/dynamic_model/model_name/activate 1/0
```
and reset to initial position of trajectory by calling service 
```
rosservice call /gazebo/dynamic_model/model_name/reset 
```

The trajectory file can be loaded by publishing path to trajectory file on topic `/gazebo/dynamic_model/model_name/load_map`.

The current state of the model can be obtained by subscribing topic `/gazebo/dynamic_model/model_name/odometry`.

