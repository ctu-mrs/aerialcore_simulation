# Aerial-Core simulation

Repository with gazebo models and worlds for Aerial-Core project.

## World configuration

The procedural power line spawning is linked with Gazebo topic `/gazebo/default/pose/info`. Gazebo publishes position and orientation of each UAV part in the simulation allowing to generate the world with respect to the specified unit. The default world file(`/worlds/aerial_core_autospawn.world`) incorporate configurable parameter`<uav_name>` setting the reference unit for world spawning. The default value is set to `uav1` for default CTU-MRS simulation. You can set the value to fit your simulation UAV setup.
