# aeb
From package '[aeb_ros](https://github.com/ISC-Project-Phoenix/aeb_ros)'
# File
`./src/aeb.cpp`

## Summary 
 Lidar based Automatic emergency braking node for ackermann drive vehicles.

See [here](https://github.com/ISC-Project-Phoenix/design/blob/main/software/embed/AEB.md) for algorithm design information.

## Topics

### Publishes
- `/aeb_out`: Publishes messages containing the time to collision if a collision under the configured TTC is predicted.

### Subscribes
- `/odom`: Odom used to derive speed and steering info.
- `/scan`: Laserscans to build the occupancy grid with. This may be incremental scans or complete scans.

## Params
- `wheelbase`: Wheelbase of the vehicle. Default 1.8
- `lidar_frame`: Frame of the source laserscans. Default "laser_link"
- `min_ttc`: Minimum allowed Time To Collision (a TTC under this will trigger braking), in seconds. Default 2
- `step_size`: Size of a step in forward prediction, in ms. The higher this value, the lower the precision of predicted collision time,
but the better the performance. Note that this doesn't effect the accuracy of collision detection.
Default 50

- `scan_region_start`: Start of desired scan region, in degrees. Scans outside of this region will be ignored, but incremental scans that land
partially in this area will be included. If your lidar gives full scans on /scan, then this param is ignored. Such is the
case in gazebo. Default 360 - 25

- `scan_region_end`: End of desired scan region, in degrees. Scans outside of this region will be ignored, but incremental scans that land
partially in this area will be included. If your lidar gives full scans on /scan, then this param is ignored. Such is the
case in gazebo. Default 25

- `collision_box`: Collision box of the vehicle, defined as the top left and bottom right of the rectangle with repect to base_link.
This param is a string in the format [[float, float], [float, float]].
Default [[-0.675, 1.43],[0.675, -0.59]]


