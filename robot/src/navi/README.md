# navigation package

## 概况

* 导航功能包，实现机器人自主导航功能。
  * 支持自定义路径
  * 支持预设路径导航
  * 支持即时路径导航
  * 支持动态避障功能
* 路径选取：通过话题`/waterplus/save_waypoints`传递路径点信息
* 导航运动：通过话题`/fight_with_hair/basic_move/vel`向基础运动模块发送运动指令

## 使用方法

### 预设路径

* 加载地图文件以供标记

  ```shell
  roslaunch navi add_waypoint.launch
  ```

* 在RViz中使用`Add Waypoint`选择路径点

* 保存路径点文件

  ```shell
  rosrun navi way_saver /path/to/the/waypoint/file
  ```

  > 路径点文件默认命名为`waypoints.xml`

### 虚拟仿真

* 开启导航功能

  ```shell
  roslaunch navi navi_sim.launch
  ```

* 添加途径点

  * 使用RViz工具栏中的`Add Waypoint`添加导航点
  
    > 注：如果没有可点击工具栏**最右侧**的加号选择

  * 设置好任一导航点后机器人**立即**开始运动

    > 如果希望机器人在**特定时间**才开始移动，注释掉launch文件中以下行。
  >
    > ```xml
  > <node name="wp_nav_test" pkg="waterplus_map_tools" type="wp_nav_test"/>
    > ```
    >
  
* 开始巡航

  ```shell
  rosrun waterplus_map_tools wp_nav_test
  ```

  > 在机器人运动过程中可以继续添加导航点，将会按顺序逐一规划路径。

### 真机运行

和虚拟仿真基本相同，差异仅在调用的launch文件上。

```shell
roslaunch navi navi_gen.launch
```

## 注意事项

* 若gazebo仿真时场景显示不全，很有可能是`wpb_sim/worlds`文件夹下的场景文件出现了路径问题，修改文件中`<geometry>`下`<mesh>`标签路径即可。

  ```xml
  <geometry>
     <mesh>
     <uri>/home/your_username/your_project_name/src/wpb_sim/meshes/tea_table.dae</uri>
     </mesh>
  </geometry>
  ```

* 初次运行时可能出现雷达包缺失的情况，需要在本机ROS中安装雷达依赖包。

  ```shell
  sudo apt-get install ros-kinetic-rplidar-ros
  ```


## 开源包依赖

* `waterplus_map_tools`：提供灵活的自定义地图导航工具
* `wpb_home`：提供机器人所需驱动和配置文件
* `wpb_sim`：提供机器人仿真环境