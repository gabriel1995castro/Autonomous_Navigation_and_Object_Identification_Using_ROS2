# T2: Autonomous Navigation and Object Identification Using ROS 2.

This project focuses on developing an autonomous robotic system capable of:

- Implement **autonomous navigation** in a partially known environment.
- Detect and classify **two types of obstacles**: **spheres and boxes**.
- Record **position, dimensions, and total count** of detected obstacles.
- Ensure full environment coverage and generate a **detailed report** of detected objects.
  
## Tools

- **Programming Language:** Python (ROS 2)
- **Simulation:** Gazebo
- **Robot Model:** TurtleBot3 - Burger with LiDAR and odometry sensors

## System Requirements

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Gazebo 11.10.2
- Python 3.10.12

## Navigation and Obstacle Detection

The `robot_nav6.py` node uses LiDAR sensors to detect obstacles and adjust the robot's trajectory. It works as follows:

1. LiDAR takes readings of the area around the robot, capturing distances to objects.

2. If an obstacle is detected at a distance smaller than the safe limit (safe_distance = 0.5m), the robot changes its direction to avoid collisions.

3. The system also maps visited areas, dividing the environment into square zones and recording the explored coordinates.

4. LiDAR takes readings of the area around the robot, capturing distances to objects in a 360-degree field of view.

5. To detect obstacles ahead, the system considers a central cone of 45 degrees, analyzing only the points within this region.

6. If an obstacle is detected at a distance smaller than the safety limit (safe_distance = 0.5m), the robot stops moving forward and rotates until it finds a safe direction.

7. If no obstacles are detected within the monitored zone, the robot continues to advance in a straight line.

8. Furthermore, the robot changes direction randomly after a variable time interval (between 7 and 20 seconds), ensuring an efficient exploration pattern and avoiding getting stuck in narrow areas.

9. The system also maps visited areas, dividing the environment into square zones and recording the explored coordinates.

The `object_detector_node.py` node is responsible for detecting objects based on data provided by the LiDAR sensor and the robot's odometry. It processes environmental data, identifies groupings of points and classifies objects.

### LiDAR Data Acquisition and Processing

LiDAR emits laser beams and returns the distance to objects around the robot and `object_detector_node.py` collects these measurements and converts the data to Cartesian coordinates relative to the robot's current position.

#### Data Conversion

Each LiDAR reading contains a set of distances associated with specific angles. These readings are converted to 2D coordinates using:

$$
x = d \cdot \cos(\theta)
$$

$$
y = d \cdot \sin(\theta)
$$

where:

- $d$ is the distance measured by LiDAR.
- $\theta$ is the corresponding angle.
    
The resulting points are stored in a list for later processing.

### Object Segmentation and Identification

The node uses the DBSCAN (Density-Based Spatial Clustering of Applications with Noise) algorithm to group the detected points. This method allows you to identify dense regions of points, assuming that they belong to the same object.
 
**DBSCAN steps:**

1. Defines a search radius and a minimum number of points to form a group.
2. Groups nearby points that meet these criteria.
3. Ignores isolated points as noise.

The result is a set of clusters representing distinct objects.

### Classification of Objects
  
After identifying groupings, the node analyzes their geometric shapes to classify the objects:

**Adjustment by RANSAC (Random Sample Consensus):**
1. The algorithm tries to fit the cluster points to a geometric model.
2. If the points can be fit to a circle within a margin of error, the object is classified as a "Sphere".
Otherwise, the object is classified as "Box".

**Distance Filter:**
Only objects within the range of 0.2m to 1.8m are considered valid for registration.


### Environment Mapping

The system uses odometry information to divide the environment into zones of predefined size (zone_size = 4). Each zone is registered as the robot moves, allowing you to monitor the area already covered.

![Screenshot from 2025-02-17 22-43-53](https://github.com/user-attachments/assets/7f39e7bc-c4ba-49e4-b65d-eac7bcda3220)

If the robot goes more than 20 seconds without identifying a new explored area, it signals the end of exploration and stops its navigation.

### Exploration Stop

The system has a stopping mechanism based on a topic (/stop_flag). When a stop signal (Bool = True) is received, the robot stops its movement and ends exploration. This command occurs when the robot finds no new areas to explore after 20 seconds or if the obstacle count remains constant for 1 minute.

## Testing

To simplify the testing process, the robot used will be introduced into the world in question automatically from the launch file. The TurtleBot3 Burger robot is loaded into the Gazebo environment in a random position at each simulation run. The position and orientation of the robot are dynamically generated at launch, using random values ​​for the x, y coordinates and yaw angle.

How positioning is done:

**Position (x, y)**:
The robot's coordinates at the base of the simulation environment are randomly generated within a defined range:
    
- x: Random value between -1.0 and 1.0 meters.
- y: Random value between -1.0 and 1.0 meters.

This approach ensures that the robot is positioned in different locations for each execution, increasing the variability and realism of the simulation.

**Guidance (yaw)**:
The robot's orientation angle is also randomly generated in the range -π to +π radians. This value represents the rotation of the robot around the vertical axis, and the randomness in this value ensures that the robot can start the simulation with different orientations, creating more dynamic scenarios.

### Cloning repository 

```bash 
git clone https://github.com/gabriel1995castro/autonomous_robots.git
cd autonomous_robots
```

### Building the package

```bash 
colcon build --packages-select robot_controller
source install/setup.bash
```

To start the navigation and obstacle detection system, both programs must be called via a launch file. This file ensures that the two modules run together and are correctly synchronized.

Loading the world A:

```bash 
ros2 launch robot_controller world_A_launch.py
```
Loading the world B:

```bash 
ros2 launch robot_controller world_B_launch.py
```

Execute the navigation and detectetion:

```bash 
ros2 launch robot_controller robot_controller_launch.py
```



