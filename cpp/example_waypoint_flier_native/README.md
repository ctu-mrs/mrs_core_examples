# WaypointFlier Native - ROS example

This package was created as an example of how to write a native ROS component (nodelet).
You can test the program in simulation (see our [simulation tutorial](https://ctu-mrs.github.io/docs/simulation/howto.html)).

## Functionality

* Once activated, the component/ nodelet will command an UAV to fly through random waypoints
* Service `start_waypoint_following` will activate the nodelet
* The area in which random waypoints are generated is configurable with a separate config file See [.yaml files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html)

## How to start

```bash
./tmux/start.sh
```

Then, call the services prepared in the terminal window either by:

1. Pressing tmux binding (`Ctrl + b` or `Ctrl + a`)
2. Pressing the down arrow to change to the terminal below
3. Pressing the up arrow to bring up the prepared terminal command

Or typing the following command into a terminal connected to the ROS server:
```
ros2 service call /uav1/waypoint_flier_native/start /std_srvs/srv/Trigger {}
```

## Package structure

See [ROS packages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

* `src` directory contains all source files
* `launch` directory contains `.py` files which are used to parametrize the nodelet. Command-line arguments, as well as environment variables, can be loaded from the launch files, the nodelet can be put into the correct namespace (each UAV has its namespace to allow multi-robot applications), config files are loaded, and parameters passed to the nodelet. See [.py files](https://docs.ros.org/en/foxy/How-To-Guides/Launching-composable-nodes.html)
* `include` directory contains necessory
* `config` directory contains parameters in `.yaml` files. See [.yaml files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html)
* `package.xml` defines properties of the package, such as package name and dependencies. See [package.xml](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1/Migrating-Package-XML.html)

## More complex example
* To see a similar node with more functionality and features, see [waypoint_flier](https://github.com/ctu-mrs/mrs_core_examples/tree/ros2/cpp/waypoint_flier)
