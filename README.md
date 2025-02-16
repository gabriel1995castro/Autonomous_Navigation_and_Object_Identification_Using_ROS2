# T2: Autonomous Navigation and Object Identification Using ROS 2.

## Objectives 

This project focuses on developing an autonomous robotic system capable of:

- Implement **autonomous navigation** in a partially known environment.
- Detect and classify **two types of obstacles**: **spheres and boxes**.
- Record **position, dimensions, and total count** of detected obstacles.
- Ensure full environment coverage and generate a **detailed report** of detected objects.
  
## Tools & Technologies

- **Programming Language:** Python (ROS 2)
- **Simulation:** Gazebo
- **Robot Model:** TurtleBot3 - Burger with LiDAR and odometry sensors

## Repository Structure

📂 project-root ├── src/ # Source code for navigation and perception │ ├── navigation.py # Autonomous navigation logic │ ├── perception.py # Object detection and classification │ ├── logging.py # Data recording module ├── worlds/ # Custom simulation environments ├── data/ # Output logs and detected objects ├── README.md # Documentation ├── requirements.txt # Required dependencies
