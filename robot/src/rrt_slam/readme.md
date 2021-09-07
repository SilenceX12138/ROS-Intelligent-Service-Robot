# fight_with_hair_rrt_slam

RRT_SLAM自动建图功能包，本包包含以下内容：



### gmapping线上仿真运行方法

```sh
$ roslaunch rrt_slam rrt_slam_sim.launch
```

使用Publish Point设置五个目标地点以后（最好是四个成矩形，一个在机器人附近）使用2D Nav Goal给机器人指出移动方向。

在建立地图完成之后，可以调用`map_server`服务保存建立的地图

```sh
$ rosrun map_server map_saver -f map
```

`-f`参数决定保存地图的名称，地图保存形式为`map.yaml和map.pgm`形式保存路径为运行该命令行的路径

### gmapping真机运行方法

```sh
$ roslaunch rrt_slam rrt_slam.launch
```

在真实上机时，选择该方法，来运行建图模块

使用Publish Point设置五个目标地点以后（最好是四个成矩形，一个在机器人附近）使用2D Nav Goal给机器人指出移动方向。