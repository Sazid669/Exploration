# Turtle Controller ROS Package

This package contains Python-based ROS nodes and launch files for pose control and autonomous exploration using TurtleBot3. It includes a custom PID controller, frontier-based exploration, and visualization tools.

---

## 📁 Package Structure

```
turtle_controller/
├── package.xml                     # Package manifest
├── CMakeLists.txt                  # Build configuration
├── scripts/                        # Python nodes
│   ├── custom_frontier_explorer.py
│   ├── initialpose_to_target.py
│   └── pid_pose_controller.py
├── launch/                         # Launch files
│   ├── test_navigation.launch
│   ├── autonomous_exploration.launch
│   └── pid_pose_controller.launch
├── config/                         # Costmap and planner configurations
│   ├── global_costmap_params_burger.yaml
│   ├── local_costmap_params_burger.yaml
│   ├── costmap_common_params_burger.yaml
│   ├── dwa_local_planner_params_burger.yaml
├── rviz/                           # RViz configuration
│   └── pose_controller.rviz
```

---

## Launch Instructions

### 1. **Pose Controller**

```
roslaunch turtle_controller pid_pose_controller.launch
```

### 2. **Autonomous Exploration**

```
roslaunch turtle_controller autonomous_exploration.launch
```

### 3. **Test Navigation**

```
roslaunch turtle_controller test_navigation.launch
```

---

## Dependencies

* ROS Noetic / Melodic (depending on your setup)
* `turtlebot3_navigation`
* `move_base`
* `amcl`, `map_server`, `rviz`

Ensure that TurtleBot3 is properly set up:

```bash
export TURTLEBOT3_MODEL=burger
```

---

## Maintainer

**Mir Mohibullah Sazid**
Master in Robotics - Erasmus Mundus (IFRoS)\
Email: \[[gmail](mailto:sazidarnob@gmail.com)]

Feel free to contribute, raise issues, or fork the repo!

## Demo Videos

Here are the demonstration videos of the project:

[![Video 1](https://img.youtube.com/vi/xcLszPdzBxE/0.jpg)](https://youtu.be/xcLszPdzBxE)  
**Video 1:** [https://youtu.be/xcLszPdzBxE](https://youtu.be/xcLszPdzBxE)

[![Video 2](https://img.youtube.com/vi/-wi0sF9BfeA/0.jpg)](https://youtu.be/-wi0sF9BfeA)  
**Video 2:** [https://youtu.be/-wi0sF9BfeA](https://youtu.be/-wi0sF9BfeA)

[![Video 3](https://img.youtube.com/vi/2NGsgKrl_5I/0.jpg)](https://youtu.be/2NGsgKrl_5I)  
**Video 3:** [https://youtu.be/2NGsgKrl_5I](https://youtu.be/2NGsgKrl_5I)

---

## 🔗 Related

* [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
* [ROS Navigation Stack](http://wiki.ros.org/navigation/)
