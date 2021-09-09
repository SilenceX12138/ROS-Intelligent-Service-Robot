# navigation package

## Overview

* Navigation function package to realize the autonomous navigation function of the robot.
  * Support custom path
  * Support preset path navigation
  * Support real-time route navigation
  * Support dynamic obstacle avoidance function
* Path selection: pass the waypoint information through the topic `/waterplus/save_waypoints`
* Navigation movement: send movement instructions to the basic movement module through the topic `/fight_with_hair/basic_move/vel`

## Instructions

### Default path

* Load map file for marking

  ```shell
  roslaunch navi add_waypoint.launch
  ```

* Use `Add Waypoint` to select waypoints in RViz

* Save path point file

  ```shell
  rosrun navi way_saver /path/to/the/waypoint/file
  ```

  > The path point file is named `waypoints.xml` by default

### virtual reality

* Turn on the navigation function

  ```shell
  roslaunch navi navi_sim.launch
  ```

* Add way point

  * Use `Add Waypoint` in the RViz toolbar to add navigation points
  
    > Note: If there is no option, click the plus sign on the rightmost side of the toolbar to select

  * After setting any navigation point, the robot **immediately** starts to move

    > If you want the robot to start moving at a **specific time**, comment out the following line in the launch file.
  >
    > ```xml
  > <node name="wp_nav_test" pkg="waterplus_map_tools" type="wp_nav_test"/>
    > ```
    >
  
* Start cruise

  ```shell
  rosrun waterplus_map_tools wp_nav_test
  ```

  > You can continue to add navigation points while the robot is moving, and the path will be planned one by one in order.

### Real machine operation

It is basically the same as virtual simulation, the difference is only in the called launch file.

```shell
roslaunch navi navi_gen.launch
```

## Precautions

* If the scene is not displayed completely during gazebo simulation, it is very likely that the scene file in the `wpb_sim/worlds` folder has a path problem, just modify the file path under the `<geometry>` label `<mesh>`.

  ```xml
  <geometry>
     <mesh>
     <uri>/home/your_username/your_project_name/src/wpb_sim/meshes/tea_table.dae</uri>
     </mesh>
  </geometry>
  ```

* The radar package may be missing during the first run, and the radar dependency package needs to be installed in the native ROS.

  ```shell
  sudo apt-get install ros-kinetic-rplidar-ros
  ```


## Open source package dependency

* `waterplus_map_tools`: Provide flexible custom map navigation tools
* `wpb_home`: Provide drivers and configuration files required by the robot
* `wpb_sim`: Provide a robot simulation environment