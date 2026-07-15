# tare_planner

TARE exploration planner for ground vehicles (ROS1 catkin package).

## Dependencies

- ROS Noetic + catkin
- PCL, Eigen3, OpenCV
- Bundled: `or-tools/`, `include/nlohmann/json.hpp`

## Build

```bash
cd ~/catkin_ws/src
ln -s /path/to/tare_planner .
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

## Run

```bash
roslaunch tare_planner tare_planner_indoor.launch
```
