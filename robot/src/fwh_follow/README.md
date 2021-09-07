## fwh_follow

本软件包为跟随模块，主要包括一个cpp源文件：

- `fwh_follow_yz`，用于控制跟随功能包的启动与暂停

### 介绍

- #### fwh_follow

  打开该模块即可启动跟随功能，跟随距离默认为0.6m，用户可通过前端进行控制，使跟随功能暂停或者继续。启动信号通过`/fight_with_hair/uito/follow_begin`传递给模块，消息类型是UInt32，用于暂停后继续跟随；暂停信号通过`/fight_with_hair/uito/follow_stop`传递给模块，消息类型是UInt32，用于暂停跟随。

### 使用方法

直接启动launch文件。

```sh
$ roslaunch fwh_follow fwh_follow_l.launch
```

前端进行测试时只需要启动testlaunch文件。

```sh
$ roslaunch fwh_follow fwh_follow_t.launch
```

### 注意事项

需要配备好kinect2驱动文件

