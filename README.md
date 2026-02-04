# Robotics Simulation Challenge – Navigation & Obstacle Avoidance
**Company:** Lebotics  
**Role:** Robotics Software Engineer Intern (Simulation)  
**Candidate:** Tala Chehade

---

## 1. Project Objective
The goal of this project is to demonstrate autonomous navigation and obstacle avoidance using a TurtleBot3 Waffle in a simulated Gazebo environment. The robot is required to navigate from a starting point to a goal while successfully avoiding both static and dynamic obstacles.

## 2. Environment & Requirements
To run this project, you need a system with:
* **OS:** Ubuntu 22.04
* **ROS2 Distro:** Kilted
* **Simulator:** Gazebo
* **Required Packages:** * `turtlebot3_gazebo`
    * `turtlebot3_navigation2`
    * `nav2_bringup`
* **Dependencies:**
  ```bash
  sudo apt update
  sudo apt install ros-kilted-navigation2 ros-kilted-nav2-bringup ros-kilted-turtlebot3*


## 3. Implementation Approach
I chose **Option A: Use Nav2 to send a goal and navigate in simulation**. 
* **Global Planner:** Used to find the optimal path from Start to Goal.
* **Local Planner (DWA):** Used to detect obstacles via LiDAR and steer the robot in real-time.
* **Map:** A static map was generated using SLAM and is provided in the `maps/` directory.

## 4. Navigation Scenarios (As seen in the Video)
I have demonstrated three distinct behaviors:
1.  **Static Navigation:** The robot follows a path in the standard `turtlebot3_world` layout.
2.  **Dynamic Obstacle Avoidance:** A new obstacle is added to the Gazebo world manually. The robot's **Local Costmap** updates in real-time, allowing it to "see" and steer around the object.
3.  **Path Replanning:** An obstacle is placed directly on the robot's intended path. The robot stops, recalculates a new **Global Path**, and proceeds to the goal via a different route.

## 5. How to Run
**Repository Structure**

Autonomous_Robot/

├── maps/

│   ├── my_map.yaml

│   └── my_map.pgm

├── README.md

1.  **Clone this repository** into your home directory or ROS2 workspace.
  ```bash
  git clone [https://github.com/TalaChehade04/Autonomous_Robot.git](https://github.com/TalaChehade04/Autonomous_Robot.git)
  cd Autonomous_Robot
  ```
2.  **Launch the simulation and navigation:**
   ```bash
    #Terminal 1: Launch the Simulation

    export TURTLEBOT3_MODEL=waffle

    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

    #Terminal 2: Launch Navigation
   
    export TURTLEBOT3_MODEL=waffle

    #Replace 'my_map.yaml' with your actual filename if different

    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$(pwd)/maps/my_map.yaml
```
    
## 6. Metrices & Results
* **Success Rate:** 100% (3/3 trials)
* **Avg Time to Goal:** ~12 seconds (measured using RViz navigation feedback timestamps)
* **Collisions:** 0

## 7. Assumptions & Future Improvements
* **Assumptions:** It is assumed the robot starts at the `[0,0]` coordinate or is manually localized using the "2D Pose Estimate" tool in RViz.
* **Known Limitations:** In very tight spaces, the local planner may struggle with oscillations.
* **Future Improvements:** I would implement a **Recovery Behavior** script to help the robot back up if it gets stuck in a corner.
  
## 8. Reuse vs Custom Work

### Reused Components
- TurtleBot3 Waffle model
- Nav2 stack (planner, controller, costmaps)
- Gazebo simulator

### Custom Configuration & Work
- Generated a custom map using Cartographer SLAM
- Tuned navigation parameters for obstacle avoidance
- Designed and tested multiple navigation scenarios
- Verified dynamic obstacle handling and replanning
- Structured launch and run workflow for reproducibility

## 9. Video Demo Link
https://drive.google.com/file/d/1z1DA2aPxBvgd-hG-pKHpKHRmqurIPFiw/view?usp=sharing

## 10. References & Open-Source Packages
This project was developed using open-source tools and official ROS2 documentation. All core algorithms and frameworks were reused as intended, while configuration, testing, and scenario design were manually implemented.

**ROS2 & Core Tools**

**ROS2 Documentation**
https://docs.ros.org

Used as the primary reference for ROS2 concepts, workspace setup, and CLI tools.

**ROS2 Kilted Distribution**
https://docs.ros.org/en/kilted

Official documentation for the ROS2 distribution used in this project.

**Navigation & Autonomy**

**Navigation2 (Nav2) Stack**
[https://navigation.ros.org](https://docs.ros.org/en/kilted/p/navigation2/)

Used for autonomous navigation, global and local planning, costmaps, and recovery behaviors.

**Robot Model & Simulation**

**TurtleBot3 Official Documentation**
https://emanual.robotis.com/docs/en/platform/turtlebot3/

Used for robot model setup, sensor configuration, and simulation parameters.

**TurtleBot3 Navigation Examples**
https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/

Referenced for Nav2 launch and configuration with TurtleBot3.

**Gazebo Simulator**
https://gazebosim.org

Used as the physics-based simulator for the robot and environment.

**Mapping & Localization**

**Google Cartographer (ROS2)**
https://google-cartographer.readthedocs.io/

Used for SLAM to generate the static map provided in the maps/ directory.

**Cartographer ROS Integration**
https://google-cartographer-ros.readthedocs.io/

Referenced for mapping workflow and map generation.

**Visualization & Debugging**

**RViz2 Visualization Tool**
[https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-User-Guide.html](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)

Used for visualization, goal setting, localization, and monitoring navigation behavior.
