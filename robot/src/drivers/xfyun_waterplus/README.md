# xfyun语音识别(kinetic版本)

## 使用步骤

1. 安装ROS(kinetic/Ubuntu 16.04). [安装步骤](http://wiki.ros.org/kinetic/Installation/Ubuntu)
2. 配置好开发环境. [配置方法](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. 安装依赖项:
```
sudo apt-get install ros-kinetic-audio-common
sudo apt-get install libasound2
sudo apt-get install ros-kinetic-sound-play
```
4. 获取源码:
```
cd ~/catkin_ws/src/
git clone https://github.com/6-robot/xfyun_kinetic.git
```
5. 编译
```
cd ~/catkin_ws
catkin_make
```

## 平台介绍
xfyun语音识别是[北京六部工坊科技有限公司](http://www.6-robot.com)为旗下WP系列机器人设计的语音识别工具,使用的是科大讯飞的云识别引擎。目前支持启智ROS和启明1服务机器人。

## 操作方法

### 1. 中文语音识别
先连接互联网,使用如下指令开始识别中文:
```
roslaunch xfyun_waterplus iat_cn.launch
```
### 2. 英文语音识别
先连接互联网,使用如下指令开始识别英文:
```
roslaunch xfyun_waterplus iat_en.launch
```
### 3. 启明1脚本
对于[启明1服务机器人](https://github.com/6-robot/wpr1),使用如下脚本启动语音识别引擎:
```
roslaunch wpr1_tutorials speech_recognition.launch
```

