# Sweeping Generator

## How to start

```bash
./tmux/start.sh
```

After taking off, call the service prepared in the bottom terminal window:
```bash
ros2 service call /$UAV_NAME/sweeping_generator/start mrs_msgs/srv/Vec1 "{goal: 3.0}"
```
