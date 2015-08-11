### Shopping Scenario Meta Package

This package holds shopping scenario related code, models, and
documentation.

To run it (after the robot performing the shopping tasks was started
and localized), just do

```bash
$ roslaunch shopping_scenario_executive shopping_scenario.launch
```

and it should start all necessary components. To run the Gazebo
simulation of the scenario (with all models spawned etc.), run

```bash
$ roslaunch shopping_scenario_executive shopping_scenario_simulation.launch
```

The simulated part requires to have the package
[`nav_pcontroller`](https://github.com/code-iai/nav_pcontroller)
present and compiled in the ROS package path.


### The Structure Explained

The following contents (sub-directories) are held within this meta
package:

 * `shopping_scenario`: Helper directory for the meta package, only
   holding its `CMakeLists.txt` and `package.xml`.

 * `shopping_scenario_executive`: The top- and high-level plans that
   control the shopping scenario behavior of the controlled robot are
   stored here. Also, the executable entry point is in this package.

 * `shopping_scenario_gazebo`: A gazebo world definition for
   simulating the scenario. This is being developed in the context of
   continuous integration efforts (first simulate, then execute on the
   real robot).

 * `shopping_scenario_maps`: Floor map for the robot to localize on,
   as well as global coordinate origin information.

 * `shopping_scenario_models`: Furniture, room, and object models for
   simulation, object detection (perception), reasoning, and
   visualization.

 * `shopping_scenario_reasoning`: Reasoning components for giving the
   robot semantic access to the environment (refer to furniture by
   name, know invisible object characteristcs / static background
   knowledge).

 * `pr2_moveit_node`: Readily configured MoveIt! node to control the
   simulated PR2 in Gazebo.

 * `attache_msgs`: Message and service definitions for using the
   `libattache.so` plugin for Gazebo (attachment and detachment of
   objects to each other, used for simulated grasping).


### Open To Dos

The following points are to be done in the near future:

 * Append handle-information to the objects in
   `shopping_scenario_models`.
