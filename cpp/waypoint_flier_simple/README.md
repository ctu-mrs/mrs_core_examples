# WaypointFlier Simple - ROS simple example

This package was created as an example of how to write a very simple ROS nodelet.
You can test the program in simulation (see our [simulation tutorial](https://ctu-mrs.github.io/docs/simulation/howto.html)).

## Functionality

* Once activated, the nodelet will command an UAV to fly through random waypoints
* Service `start_waypoint_following` will activate the nodelet
* The area in which random waypoints are generated is configurable with a separate config file See [.yaml files](http://wiki.ros.org/rosparam)

## How to start

```bash
./tmux/start.sh
```

The call the services prepared in the terminal window.

## Package structure

See [ROS packages](http://wiki.ros.org/Packages)

* `src` directory contains all source files
* `launch` directory contains `.launch` files which are used to parametrize the nodelet. Command-line arguments, as well as environment variables, can be loaded from the launch files, the nodelet can be put into the correct namespace (each UAV has its namespace to allow multi-robot applications), config files are loaded, and parameters passed to the nodelet. See [.launch files](http://wiki.ros.org/roslaunch/XML)
* `config` directory contains parameters in `.yaml` files. See [.yaml files](http://wiki.ros.org/rosparam)
* `package.xml` defines properties of the package, such as package name and dependencies. See [package.xml](http://wiki.ros.org/catkin/package.xml)

## More complex example

* To see a similar node with more functionality and features, see [example_ros_uav](https://github.com/ctu-mrs/example_ros_uav)
