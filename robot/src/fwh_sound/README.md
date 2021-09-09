## fwh_sound

This software package is a speech recognition module, which mainly includes a cpp source file:

-`fwh_sound_yz.cpp`, used for voice command recognition

### introduce

-#### fwh_sound

   Turn on the module to start the voice recognition function. The supported voice commands include target recognition (`obj`), navigation (`navi`), control (`mani`), follow (`follow`), and map (`build`) ), the module sends the recognized command to the front end in the form of a string through `/fight_with_hair/toui/sound_ins`, and the front end sends an end signal to the module through `fight_with_hair/uito/state_sound` after the corresponding command is executed. Indicates that voice recognition is allowed to pass the next instruction.

### Instructions

Start the launch file directly.

```sh
$ roslaunch fwh_sound fwh_sound_l.launch
```

Only need to start the testlaunch file when the front-end tests.

```sh
$ roslaunch fwh_sound fwh_sound_t.launch
```