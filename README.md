# Kobuki_Navigation
This project has been completed as part of a coursework for COMPSYS 732 at the University of Auckland.
<br/><br/>

## Testing Environment
Ubuntu 20.04 LTS <br/>
ROS1 Noetic <br/>
Simulated via Gazebo & RViz
<br/><br/>
Run via
```
source devel/setup.bash
roslaunch kobuki_navigation master.launch
```
If gets spammed with TF_REPEATED_DATA warning (a known bug in AMCL library), run
```
source devel/setup.bash
roslaunch kobuki_navigation master.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
```
<br/>

## Robot Model
This project utilised a kobuki (turtlebot 2) mobile base, with Microsoft Kinect as the perception sensor for its surroundings.
<br/><br/>
## Project Description
In this blind navigation project, the primary objective was to explore an unfamiliar environment, locate a specific goal object, and return to the starting location. 
The goal object in this scenario was a 10 cm x 10 cm x 10 cm red cube, generated within Gazebo with its default red texture. 

To achieve this goal reliably, this project was divided into three main challenges of:
1. Simultaneous Localisation and Mapping (SLAM)
2. Navigation
3. Goal object search

### Online SLAM - GMapping vs RTABMap
Starting off with the SLAM component, two different algorithms were tested and compared, which were improved Rao-Blackwellised Particle Filter (iRBPF) and Real-Time Appearance-Based (RTAB) loop closure detection.
Both algorithms were reliably implemented through an existing ROS library of GMapping and RTABMap respectively. 
<br/><br/>
Over ten trials, a TurtleBot 2 with an RGB-D camera attempted to map the entire Demo World Two. 
For consistency, the FBE approach was employed using navigation algorithms, specifically the DWA local planner and Dijkstra's algorithm as the global planner. 
On average, the robot equipped with the iRBPF algorithm via the GMapping library accurately mapped the area and localised itself within an error range of ±0.5 unit in the x and y directions ZERO times. 
In comparison, the robot using the RTAB Mapping algorithm via the RTAB-Map package achieved this accuracy NINE times.
<br/><br/>
The high error rate for the iRBPF method can be attributed to the narrow field of view and restricted input. 
The GMapping library used for the iRBPF method only accepted 2D LiDAR and odometry data, which was simulated via the depth information from the RGB-D camera. 
This approach limited the field of view to only 60 degrees towards the front direction. 
It also restricted the detection of unique landmark feature points due to limited data compared to the full RGB-D dataset generated.<br/>
On the other hand, the satisfactory result yielded by the RTABMap algorithm was expected by using a larger dataset with higher computational cost to fulfil the SLAM requirement. 
Proven by the unmatched success rate between the two tested SLAM algorithms and the lack of restriction on the computational cost, I have concluded that RTAB loop closure detection is the best SLAM solution for this project.
<br/><br/>

### Navigation – Global Planner (Dijkstra's Algorithm) + Local Planner (DWA)
The navigation challenge involved testing Dijkstra's and D* algorithms as global planners and the Dynamic Window Approach (DWA) and Timed Elastic Band (TEB) as local planners. 





Two strategies were evaluated for the goal object search: Random Obstacle Avoidance (ROA) and Frontier-Based
Exploration (FBE). This comprehensive evaluation aimed to identify the most effective methods for ensuring
reliable and efficient robot performance in navigating and searching through unknown environments.

WIP


