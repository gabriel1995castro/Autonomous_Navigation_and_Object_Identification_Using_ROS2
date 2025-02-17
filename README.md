# T2: Autonomous Navigation and Object Identification Using ROS 2.

This project focuses on developing an autonomous robotic system capable of:

- Implement **autonomous navigation** in a partially known environment.
- Detect and classify **two types of obstacles**: **spheres and boxes**.
- Record **position, dimensions, and total count** of detected obstacles.
- Ensure full environment coverage and generate a **detailed report** of detected objects.
  
## Tools & Technologies

- **Programming Language:** Python (ROS 2)
- **Simulation:** Gazebo
- **Robot Model:** TurtleBot3 - Burger with LiDAR and odometry sensors

## System Requirements

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Gazebo 11.10.2
- Python 3.10.12

## Setup

### Cloning repository 

```bash 
git clone https://github.com/gabriel1995castro/autonomous_robots.git
cd autonomous_robots
```

## Building the package

```bash 
colcon build --packages-select robot_controller
source install/setup.bash
```

## Running the Simulation

```bash 
ros2 launch robot_controller launch.py
```
