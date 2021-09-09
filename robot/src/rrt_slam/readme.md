# fight_with_hair_rrt_slam

RRT_SLAM automatic mapping function package, this package contains the following contents:



### Gmapping online simulation running method

```sh
$ roslaunch rrt_slam rrt_slam_sim.launch
```

After using Publish Point to set five target locations (preferably four in a rectangle and one near the robot), use 2D Nav Goal to point out the direction of movement of the robot.

After the map is created, you can call the `map_server` service to save the created map

```sh
$ rosrun map_server map_saver -f map
```

The `-f` parameter determines the name of the saved map, the map save format is `map.yaml and map.pgm` The save path is the path to run the command line

### How to run gmapping on real machine

```sh
$ roslaunch rrt_slam rrt_slam.launch
```

In the real computer, select this method to run the mapping module

After using Publish Point to set five target locations (preferably four in a rectangle and one near the robot), use 2D Nav Goal to point out the direction of movement of the robot.