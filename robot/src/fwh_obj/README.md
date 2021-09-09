## fwh_obj

This software package is the target capture recognition module, which mainly includes two cpp source files:

-`fwh_obj_detect.cpp`, used to obtain the identified item information, and receive the number of the target from the front end, and pass the target information to the grab
-`fwh_obj_grab.cpp`, after receiving the target information, send it to /wpb_home/grab_action.

A Msg:

* `tarobj.msg`, used to record the coordinate information of the target grab

### introduce

-#### fwh_obj

  The front end sends a start detection signal (std_msgs::bool) to the target recognition module through `fight_with_hair/uito/obj_begin`, and starts target recognition after receiving the message. The target recognition module passes the number of items received through `fight_with_hair/toui/obj_num `Send to the front-end, the message type is std_msgs::UInt32, the front-end will display the identification information to the user, the user selects the grabbed item number and sends it to the target recognition module through `fight_with_hair/uito/obj_tar`, and then the target recognition module will respond accordingly The three-dimensional coordinate information is sent to the grab module to perform grabbing.

### Instructions

Start the launch file directly.

```sh
$ roslaunch fwh_obj fwh_obj_l.launch
```

Only need to start the testlaunch file when the front-end tests.

```sh
$ roslaunch fwh_obj fwh_obj_t.launch
```

### Precautions

Need to be equipped with kinect2 driver file