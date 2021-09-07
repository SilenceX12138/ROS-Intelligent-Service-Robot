## fwh_sound

本软件包为语音识别模块，主要包括一个cpp源文件：

- `fwh_sound_yz.cpp`，用于进行语音指令识别

### 介绍

- #### fwh_sound

  打开该模块即可启动语音识别功能，支持的语音指令包括目标识别（`obj`）、导航（`navi`）、控制（`mani`）、跟随（`follow`）以及建图（`build`），该模块将识别到的指令通过`/fight_with_hair/toui/sound_ins`以字符串的形式发送给前端，前端在相应指令执行完毕后通过`fight_with_hair/uito/state_sound`发送给该模块一个结束信号，表示允许语音识别传递下一条指令。

### 使用方法

直接启动launch文件。

```sh
$ roslaunch fwh_sound fwh_sound_l.launch
```

前端进行测试时只需要启动testlaunch文件。

```sh
$ roslaunch fwh_sound fwh_sound_t.launch
```

