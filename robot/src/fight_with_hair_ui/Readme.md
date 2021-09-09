# fight_with_hair_ui package

## Overview

* Front-end function package, open the front-end function to use all robot functions
* Pass front-end messages to various topics through the qnode node, and control the movement of each module of the robot

## Instructions

### Path

* Put the entire project in the root directory, and be careful not to change the name

### start up

* Open the user interface and display the main interface

  ```shell
  roslaunch fight_with_hair_ui fight_with_hair_ui.launch
  ```

  * Perform qnode initialization, open roscore, and open the simulation interface

* Click the corresponding button to enter different modules

### Active control interface

* Call the `sendMoveMsg(x,y,z),sendArmMsg()` function of `qnode` through the front-end interface buttons, keyboard and handle to send motion instructions to the topic /fight_with_hair/basic_move/vel to control the robot movement
* Keyboard
  * Q——Left front
  * W——Front
  * E——front right
  * A——Left
  * S——Stop
  * D——right
  * Z-rear left
  * X——After
  * C-rear right
  * I——Robot arm up
  * K-Robot arm down
  * J——Robot grabbing
  * L——Robot release

### Target detection and capture interface

* Display the real-time scanning results of the robot through the front-end interface, and return to the capture target through the button
* Click the start detection button and call the `sendObjSigMsg()` function of `qnode` to send a message that the target detection starts to /fight_with_hair/uito/obj_begin
* Select the item number to be grabbed and click the item button to call the `sendObjDetectMsg()` function of `qnode` to send the item number selected by the user to /fight_with_hair/uito/obj_jar

### Voice recognition function

* Click the Start Voice Recognition button to start voice recognition

### Follow the interface autonomously

* Open the interface and the robot will automatically start following
* Click the stop following button to call the `sendFollowEndMsg()` function of `qnode` to send a follow end message to the topic/fight_with_hair/uito/follow_stop

* Click the start follow button to call the `sendFollowBeginMsg()` function of `qnode` to send a restart message to the topic /fight_with_hair/uito/follow_begin

### Scan and map interface

* Click the start map creation interface to start map creation, and the front-end will feedback the map creation status in real time
* Enter the map file name, click to save the map
  * The default path `workspace/src/navi/maps/mapname.yaml`
  * Save format `mapname.yaml` and `mapname.pgm`

### Navigation interface

* Click the select map button to select a map, only the yaml format map can be selected, and the map is in the `workspace/src/navi/maps/` directory
* Click the Load Map button to load the selected map
  * If no map is selected, `workspace/src/navi/maps/map.yaml` will be loaded by default
  * At the same time load the xml format path point file with the corresponding name in the `workspace/src/navi/waypoints/` directory
  * If the robot is not displayed on the map, it may be due to a freeze, just reload the map
* Click the button to select the target point, select the navigation point, and proceed directly to the single-point autonomous navigation after the selection is completed
* Click the Select Waypoint button to select the waypoint, and the waypoint will be displayed on the map
* Click to start cruising, the robot will autonomously plan the path according to the waypoints and start cruising
* Click Save waypoints to save all waypoints
  * The default path `workspace/src/navi/waypoints/mapname.xml`