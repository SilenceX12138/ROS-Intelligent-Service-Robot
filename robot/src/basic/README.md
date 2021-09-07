## basic

本软件包为基础移动basic模块，主要包括两个cpp源文件：

- `basic_move.cpp`，机器人基础移动模块。
- `basic_avoidance.cpp`，机器人避障模块。

一个Msg：

* `Movemsg.msg`，用户监听模块与basic_move模块交互的消息。

一个Srv：

* `IsAvoidance.srv`，basic_move模块向basic_avoidance发送避障查询的消息。

### 介绍

- #### basic_move

  该模块在话题`/fight_with_hair/basic_move/vel`中接收其他模块对机器人移动的控制指令，在进行速度限制等工作之后发布给底层话题`/cmd_vel`。在发布前，会向`basic_avoidance`模块，话题`/fight_with_hair/basic_move/avoid`发送避障查询消息，进行避障操作。

- #### basic_avoidance

  该模块实时监听`/scan`中雷达发布的距离数据，并进行存储。实时监听话题`/fight_with_hair/basic_move/avoid`发来的避障请求，当在机器人运动方向上的一定距离出现障碍物后，会根据距离障碍物的距离进行不同程度的减速，直至机器人停止运动。

### 使用方法

直接启动launch文件，即可开启基础移动部分的节点。

```sh
$ roslaunch basic basic_move.launch
```
