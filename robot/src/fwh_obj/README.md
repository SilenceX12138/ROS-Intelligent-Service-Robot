## fwh_obj

本软件包为目标抓取识别模块，主要包括两个cpp源文件：

- `fwh_obj_detect.cpp`，用于获取识别到的物品信息，并且从前端接收目标物的号码，将目标物信息传给grab
- `fwh_obj_grab.cpp`，接收到目标物信息后将其发送给/wpb_home/grab_action。

一个Msg：

* `tarobj.msg`，用于记录目标抓取物的坐标信息

### 介绍

- #### fwh_obj

  前端给目标识别模块通过`fight_with_hair/uito/obj_begin `发送一个开始检测信号（std_msgs::bool），收到消息后开始进行目标识别，目标识别模块把接收到的物品数量通过`fight_with_hair/toui/obj_num`发给前端，消息类型为std_msgs::UInt32，前端将识别信息展示给用户，由用户选择抓取的物品编号并通过`fight_with_hair/uito/obj_tar`发送给目标识别模块，然后目标识别模块将相应的三维坐标信息发给抓取模块执行抓取。

### 使用方法

直接启动launch文件。

```sh
$ roslaunch fwh_obj fwh_obj_l.launch
```

前端进行测试时只需要启动testlaunch文件。

```sh
$ roslaunch fwh_obj fwh_obj_t.launch
```

### 注意事项

需要配备好kinect2驱动文件

