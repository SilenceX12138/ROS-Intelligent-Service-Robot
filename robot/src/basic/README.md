## basic

This software package is the basic mobile basic module, which mainly includes two cpp source files:

-`basic_move.cpp`, the robot basic mobile module.
-`basic_avoidance.cpp`, the robot obstacle avoidance module.

A Msg:

* `Movemsg.msg`, user monitoring module interacts with basic_move module.

A Srv:

* `IsAvoidance.srv`, the basic_move module sends an obstacle avoidance query message to basic_avoidance.

### introduce

-#### basic_move

  This module receives the control instructions of other modules for robot movement in the topic `/fight_with_hair/basic_move/vel`, and publishes it to the underlying topic `/cmd_vel` after performing speed limiting and other tasks. Before release, an obstacle avoidance query message will be sent to the `basic_avoidance` module, topic `/fight_with_hair/basic_move/avoid`, to perform obstacle avoidance operations.

-#### basic_avoidance

  This module monitors the distance data released by the radar in `/scan` in real time and stores it. Real-time monitoring of obstacle avoidance requests from the topic `/fight_with_hair/basic_move/avoid`, when an obstacle appears at a certain distance in the direction of movement of the robot, it will decelerate to varying degrees according to the distance from the obstacle until the robot stops moving.

### Instructions

Start the launch file directly to open the nodes of the basic mobile part.

```sh
$ roslaunch basic basic_move.launch
```