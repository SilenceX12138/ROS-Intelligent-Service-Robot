## fwh_follow

This software package is a follow-up module, which mainly includes a cpp source file:

-`fwh_follow_yz`, used to control the start and pause of the follow function package

### introduce

-#### fwh_follow

   Turn on the module to start the follow function. The follow distance defaults to 0.6m. The user can control the follow function through the front end to pause or continue the follow function. The start signal is passed to the module through `/fight_with_hair/uito/follow_begin`, the message type is UInt32, which is used to continue following after a pause; the pause signal is passed to the module through `/fight_with_hair/uito/follow_stop`, the message type is UInt32, which is used to pause follow.

### Instructions

Start the launch file directly.

```sh
$ roslaunch fwh_follow fwh_follow_l.launch
```

Only need to start the testlaunch file when the front-end tests.

```sh
$ roslaunch fwh_follow fwh_follow_t.launch
```

### Precautions

Need to be equipped with kinect2 driver file