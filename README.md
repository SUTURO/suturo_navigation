# suturo_navigation

## Creating a map
Corresponding HSR documentation: [7.6.3 Map](https://docs.hsr.io/manual_en/development/base_map.html?highlight=mapping)

### Using the simulator
#### Loading the world
```bash
roslaunch suturo_bringup mapping.launch world:=world.world
```
By default *robot_pos* is set to *-x 0 -y 0 -z 0 -Y 0* it can be adjusted the same way as the world but is optional.
The world is expected to be located in the suturo_resources packages world folder.

#### Starting hector_slam
First you have to kill the pose_integrator using the follwoing command:
```bash
rosnode kill /pose_integrator
```
Now you can launch hector_slam using the provided launchfile
```bash
roslaunch mapping_hector_slam hector_slam.launch
```

#### Moving the robot
You can move the robot in the Simulation using *rqt*.
```bash
rqt
```
Add the plug-in for robot control in rqt.

[Plugins] -> [Robot Tools] -> [Robot Steering]

#### Save the map
```bash
rosrun map_server map_saver -f /path/to/save/map/map_name
```
This will create a *.pmg* image of the map and *.yaml* file containing all nessecary information for the map. Please note that the path has to exists.
In the .yaml file the path to the image is relative to the root directory of the computer it was created on. In order to avoid errors later on it is recommended to adjust this path too your needs. 
