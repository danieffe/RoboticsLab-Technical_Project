# Cooperation between a mobile robot (fra2mo) and a manipulator (iiwa) for “handover/pickup”

## Avaiable packages
* `ros2_fra2mo` 
* `ros2_iiwa`
* `ros2_kdl_package`
* `aruco_ros`
* `m-explore-ros2` 

### Exploration
To start the simulation run in a terminal:
```bash
ros2 launch ros2_fra2mo project.launch.py
```
Then you can launch the explore_lite node to navigate and map the area:
```bash
ros2 launch ros2_fra2mo fra2mo_explore.launch.py
```
Run the logic node to detect the ArUco tag, transform coordinates, and save the goal to YAML:
```bash
ros2 run ros2_fra2mo save_iiwa_pose.py
```
You can save your map by running on a terminal:
```bash
ros2 run nav2_map_server map_saver_cli -f map
```

---

### Visual Servoing task and navigation

To use the velocity controller interface required for visual servoing run this command:
```bash
ros2 launch ros2_fra2mo project.launch.py iiwa_controller:=velocity_controller command_interface:=velocity
```

To use AMCL run this command:
```bash
ros2 launch ros2_fra2mo fra2mo_amcl.launch.py
```

Then launch the navigation server and costmaps:
```bash
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
```

To start the navigation to the saved position run this command:
```bash
ros2 run ros2_fra2mo go_too_iiwa.py
```

To activate visual servoing run the the following command:
```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p ctrl:=vision_ctrl
```
