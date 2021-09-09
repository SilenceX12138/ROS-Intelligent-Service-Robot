# fight_with_hair_slam

SLAM mapping function package, this package contains the following contents:

* **gmapping_demo**: gmapping SLAM

### How to run gmapping example

```sh
$ roslaunch slam gmapping_simulate.launch
```

Control the car to move, you can see the establishment of the map

After the map is created, you can call the `map_server` service to save the created map

```sh
$ rosrun map_server map_saver -f map
```

The `-f` parameter determines the name of the saved map, the map save format is `map.yaml and map.pgm` The save path is the path to run the command line

### How to run gmapping on real machine

```sh
$ roslaunch slam gmapping.launch
```

In the real computer, select this method to run the mapping module