# fight_with_hair_slam

SLAM建图功能包，本包包含以下内容：

* **gmapping_demo**: gmapping SLAM

### gmapping示例运行方法

```sh
$ roslaunch slam gmapping_simulate.launch
```

操控小车移动，可以看到地图的建立

在建立地图完成之后，可以调用`map_server`服务保存建立的地图

```sh
$ rosrun map_server map_saver -f map
```

`-f`参数决定保存地图的名称，地图保存形式为`map.yaml和map.pgm`形式保存路径为运行该命令行的路径

### gmapping真机运行方法

```sh
$ roslaunch slam gmapping.launch
```

在真实上机时，选择该方法，来运行建图模块

