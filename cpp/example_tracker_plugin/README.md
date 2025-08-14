# Example Tracker Plugin

Example tracker plugin for the MRS Control Manager. 

Tracker plugin is a way to write a plugin for custom tracker. 
Information about already implemented tracker plugins is [here](https://ctu-mrs.github.io/docs/features/trackers/).

Tracker plugin is initialised inside [control_manager](https://github.com/ctu-mrs/mrs_uav_managers/blob/ros2/src/control_manager/control_manager.cpp).

Tracker subspace heirarchy:
```
control_manager
    - mrs_uav_trackers
        - example_tracker
```

## How to start

```bash
./tmux/start.sh
```
