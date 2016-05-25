# cam_exploration
ROS package implementing a 2D exploration framework for 3D mapping with RGBD camera(s).

It works along with the `rtabmap_ros` ROS package. More information and instructions in their [Github page](https://github.com/introlab/rtabmap_ros).

It can work with any RGBD camera (and laser scan) configuration allowed in `rtabmap_ros`. The point cloud ground projection is used for exploration purposes.

This framework allow to choose many strategic decision from command line.

### Replanning condition
When should the robot replan a new navigation goal? Many replanning conditions can be selected such that if any of them is met, a new evaluation of the frontiers is performed to choose the next exploration target.
Current conditions:
* **not_moving**: The robot is currently not heading to any goal.
* **too_much_time_near_goal**: Spending too much time near the goal. It votes for replanning if the robot has spent some time near its current goal and it's properly oriented with the goal.
* **isolated_goal**: The goal is not close to any frontier. It is activated when none of the cells in an arbitrary large neighborhood of the goal corresponds to a frontier.

### Frontier evaluation
Which is the cost function for evaluate a frontier as a exploration target? It is described by a weighted sum of frontier evaluation functions.
Current frontier evaluation functions:
* **Maximum size**: It favors larger frontiers over smaller ones.
* **Minimum euclidean distance**: It favors frontiers which are closer to the robot position, regardless of the obstacles in between.
* **Minimum A* distance**: The function favors the frontiers with shorter distances, taking as distance the path found by A* algorithm.

### Goal selection
Once a frontier is selected, where should the robot go exactly? The function to find a proper 2D navigation point from a frontier is a parameter of the ROS parameter server.
Currently one single goal selection function is used:
* **Mid point**: Selects a navigation goal from where the middle point of the frontier is visible.

The file `standard/config.yaml` contains an example file containing a possible configuration.

## Usage
To use this package `rtabmap_ros` has to be configured according to your robot. Example files are contained in the `launch` folder.
Once the package is compiled and visible for your ROS system and `rtabmap` is running, launch the `cam_exploration` node:
``` sh
roslaunch cam_exploration cam_exploration.launch
```

