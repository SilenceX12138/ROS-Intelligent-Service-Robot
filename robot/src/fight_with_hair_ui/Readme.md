# fight_with_hair_ui package

## 概况

* 前端功能包，打开前端功能可使用所有机器人功能
* 通过qnode节点向各个话题传递前端消息，控制机器人各模块运动

## 使用方法

### 路径

* 将整个工程放在根目录下，注意不要改名

### 启动

* 打开用户界面，显示主界面

  ```shell
  roslaunch fight_with_hair_ui fight_with_hair_ui.launch
  ```

  * 进行qnode初始化，打开roscore，打开仿真界面

* 点击对应按钮进入不同模块

### 主动控制界面

* 通过前端界面按钮、键盘、手柄调用`qnode`的`sendMoveMsg(x,y,z),sendArmMsg()`函数向话题 /fight_with_hair/basic_move/vel  发送运动指令，控制机器人运动
* 键盘
  * Q——左前
  * W——前
  * E——右前
  * A——左
  * S——停止
  * D——右
  * Z——左后
  * X——后
  * C——右后
  * I——机械臂向上
  * K——机械臂向下
  * J——机械臂抓取
  * L——机械臂释放

### 目标检测及抓取界面

* 通过前端界面显示机器人实时扫描结果，通过按钮返回抓取目标
* 点击开始检测按钮，调用`qnode`的`sendObjSigMsg()`函数向/fight_with_hair/uito/obj_begin发送目标检测开始的消息
* 选择抓取物品编号，并点击抓取物品按钮，调用`qnode`的`sendObjDetectMsg()`函数向/fight_with_hair/uito/obj_jar发送用户选择需要抓取的物品编号

### 语音识别功能

* 点击开始语音识别按钮开始语音识别

### 自主跟随界面

* 打开界面则机器人自动开始跟随
* 点击停止跟随按钮调用`qnode`的`sendFollowEndMsg()`函数向话题/fight_with_hair/uito/follow_stop发送跟随结束消息

* 点击开始跟随按钮调用`qnode`的`sendFollowBeginMsg()`函数向话题/fight_with_hair/uito/follow_begin发送重新开始跟随消息

### 扫描并建图界面

* 点击开始建图界面开始建图，前端实时反馈地图建立情况
* 输入地图文件名，点击保存地图
  * 默认路径 `工作空间/src/navi/maps/mapname.yaml`
  * 保存格式`mapname.yaml`和`mapname.pgm`

### 导航界面

* 点击选择地图按钮选择地图，只能选择yaml格式地图，地图在 `工作空间/src/navi/maps/`目录下
* 点击加载地图按钮加载选择的地图
  * 若未选择地图，默认加载 `工作空间/src/navi/maps/map.yaml`
  * 同时加载`工作空间/src/navi/waypoints/`目录下对应名称的xml格式路径点文件
  * 若机器人未在地图中显示，可能是由于卡顿原因，重新加载地图即可
* 点击选择目标点按钮，选择导航点，选择完毕直接进行单点自主导航
* 点击选择路径点按钮，选择路径点，路径点显示在地图上
* 点击开始巡航，机器人根据路径点自主规划路径并开始巡航
* 点击保存路径点，保存所有路径点
  * 默认路径`工作空间/src/navi/waypoints/mapname.xml`

