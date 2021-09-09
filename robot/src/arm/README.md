## arm

This software package is a mobile arm module of a robotic arm, which mainly includes a cpp source file:

-`arm_move.cpp`, robot arm movement module

A Msg:

* `armMsg.msg`, the user monitors the messages that the module interacts with the arm_move module.

### introduce

-#### arm_move

   The robot arm movement module monitors the message of the topic `/fight_with_hair/arm_move` in real time, and obtains the robot motion instructions from the user monitoring class and the target grabbing class from the topic. The message type is a custom arm::armMsg type . The robot arm class performs the message conversion acceptable to the underlying hardware on the received motion command of the robot arm, and converts the arm::armMsg message type to the sensor_msgs::JointState type acceptable to the bottom layer. The transformed sensor_msgs:: JointState type message is ready to be sent to the underlying `/wpb_home/mani_ctrl` topic for the underlying hardware related nodes to receive.

### Instructions

Start the launch file directly to open the nodes of the basic mobile part.

```sh
$ roslaunch arm arm_move.launch
```