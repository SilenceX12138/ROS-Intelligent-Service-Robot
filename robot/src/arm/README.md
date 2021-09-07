## arm

本软件包为机械臂移动arm模块，主要包括一个cpp源文件：

- `arm_move.cpp`，机械臂移动模块

一个Msg：

* `armMsg.msg`，用户监听模块与arm_move模块交互的消息。

### 介绍

- #### arm_move

  机械臂移动模块实时监听话题`/fight_with_hair/arm_move`的消息，从该话题中获取从用户监听类以及目标抓取类传递过来的机械臂运动指令，消息的类型为自定义的arm::armMsg类型。机械臂类对收到的机械臂运动指令进行底层硬件可接受的消息转换，将arm::armMsg消息类型转换为底层可接受的sensor_msgs::JointState类型。转化好的sensor_msgs:: JointState类型消息准备发送给底层的`/wpb_home/mani_ctrl`话题，供底层硬件相关节点接收.

### 使用方法

直接启动launch文件，即可开启基础移动部分的节点。

```sh
$ roslaunch arm arm_move.launch
```
