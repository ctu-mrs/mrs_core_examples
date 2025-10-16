# ExampleWaypointFlier ROS2 example

This package was created as an example of how to write ROS components.
The package is written in C++ and features custom MRS libraries and msgs.

## Functionality

* Desired waypoints are loaded as a matrix from config file
* Service `fly_to_first_waypoint` prepares the UAV by flying to the first waypoint
* Service `start_waypoint_following` causes the UAV to start tracking the waypoints
* Service `stop_waypoint_following` stops adding new waypoints. Flight to the current waypoint is not interrupted.

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
ros2 service call /$UAV_NAME/waypoint_flier/fly_to_first_waypoint std_srvs/srv/Trigger {}

For navigating between terminals use `shift + arrow keys` and for navigating between panes of terminal use `ctrl + k' to move horizontally between panes and 'ctrl + l` to move vertically.
```

## Package structure

See [ROS packages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

* `src` directory contains all source files
* `include` directory contains all header files. It is good practice to separate them from source files.
* `launch` directory contains `.py` files which are used to parametrize the components. Command-line arguments, as well as environment variables, can be loaded from the launch files, the component can be put into the correct namespace (each UAV has its namespace to allow multi-robot applications), config files are loaded, and parameters passed to the component. See [.py files](https://docs.ros.org/en/foxy/How-To-Guides/Launching-composable-nodes.html)
* `config` directory contains parameters in `.yaml` files. See [.yaml files](https://docs.ros.org/en/jazzy/How-To-Guides/Using-ros2-param.html)
* `package.xml` defines properties of the package, such as package name and dependencies. See [package.xml](https://docs.ros.org/en/eloquent/Tutorials/Creating-Your-First-ROS2-Package.html)

## Example features

* [Component](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-a-Composable-Node.html) initialization
* [Subscriber, publisher](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html), and [timer](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1TimerBase.html) initialization
* [Service servers and clients]https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html) initialization
* Loading [parameters](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html) with `mrs_lib::ParamLoader` class
* Loading [Eigen matrices](https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html) with `mrs_lib::ParamLoader` class
* Checking nodelet initialization status in every callback
* Checking whether subscribed messages are coming
* Throttling [text output](https://docs.ros.org/en/jazzy/Tutorials/Demos/Logging-and-logger-configuration.html) to a terminal
* [Thread-safe access](https://en.cppreference.com/w/cpp/thread/mutex) to variables using `std::lock_scope()`
* Using `ConstPtr` when subscribing to a topic to avoid copying large messages
* Storing and accessing matrices in `Eigen` classes
* [Remapping topics](https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html) in the launch file

## Coding style

For easy orientation in the code, we have agreed to follow the [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide) when writing our packages.
Also check out our general [C++ good/bad coding practices tutorial](https://ctu-mrs.github.io/docs/introduction/c_to_cpp.html).

### Naming variables

* Member variables are distinguished from local variables by underscore at the end:
  - `position_x` -  local variable
  - `position_x_` -  member variable
* Also, we distinguish parameters which are loaded as parameters by underscore at the beginning
* Descriptive variable names are used. The purpose of the variable should be obvious from the name.
  - `sh_odometry_` - member subscriber handler to uav odometry msg type
  - `pub_reference_` - member publisher hanadler of reference msg type
  - `srv_server_start_waypoints_following_` - member service server for starting following of waypoints
  - `ExampleWaypointFlier::timerCheckSubscribers()` - callback of timer which checks subscribers
  - `mutex_current_waypoint_` - mutex locking access to variable containing current UAV waypoint

### Good practices

* [Nodelet everything!](https://www.clearpathrobotics.com/assets/guides/ros/Nodelet%20Everything.html) Nodelets compared to nodes do not need to send whole messages. Multiple nodelets running under the same nodelet manager form one process and messages can be passed as pointers.
* Do not use raw pointers! Smart pointers from `<memory>` free resources automatically, thus preventing memory leaks.
* Lock access to member variables! Nodelets are multi-thread processes, so it is our responsibility to make our code thread-safe.
  - Use `c++17` `scoped_lock` which unlocks the mutex after leaving the scope. This way, you can't forget to unlock the mutex.
* When a component is initialized, the method `intialize()` is called. In the method, the subscribers are initialized, and callbacks are bound to them. The callbacks can run even before the `intialize()` method ends, which can lead to some variables being still not initialized, parameters not loaded, etc. This can be prevented by using an `is_initialized_`, initializing it to `false` at the beginning of `intialize()` and setting it to true at the end. Every callback should check this variable and continue only when it is `true`.
* Use `mrs_lib::ParamLoader` class to load parameters from launch files and config files. This class checks whether the parameter was actually loaded, which can save a lot of debugging. Furthermore, loading matrices into config files becomes much simpler.
* For printing debug info to terminal use `RCLCPP_INFO()`, `RCLCPP_WARN()`, `RCLCPP_ERROR()` macros. Do not spam the terminal by printing a variable every time a callback is called, use for example `RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "dog")` to print *dog* not more often than every second. Other animals can also be used for debugging purposes.
* If you need to execute a piece of code periodically, do not use sleep in a loop, or anything similar. The ROS API provides `mrs_lib::TheadTimer` (or native but greedy `mrs_lib::ROSTimer`) class for this purposes, which executes a callback every time the timer expires.
* Always check whether all subscribed messages are coming. If not, print a warning. Then you know the problem is not in your nodelet and you know to look for the problem in topic remapping or the node publishing it.
