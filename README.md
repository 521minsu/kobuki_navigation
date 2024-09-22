# Kobuki_Navigation
This project has been completed as part of a coursework for COMPSYS 732 at the University of Auckland.
<br/><br/>

## Testing Environment
```
Ubuntu 20.04 LTS 
ROS1 Noetic 
Simulated via Gazebo & RViz
```
<br/><br/>
Run via
```
source devel/setup.bash
roslaunch kobuki_navigation master.launch
```
If gets spammed with TF_REPEATED_DATA warning (a known bug in AMCL library), run via
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
<br/><br/>

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
It also restricted the detection of unique landmark feature points due to limited data compared to the full RGB-D dataset generated.
On the other hand, the satisfactory result yielded by the RTABMap algorithm was expected by using a larger dataset with higher computational cost to fulfil the SLAM requirement. 
Proven by the unmatched success rate between the two tested SLAM algorithms and the lack of restriction on the computational cost, I have concluded that RTAB loop closure detection is the best SLAM solution for this project.
<br/><br/>

### Navigation – Global Planner (Dijkstra's Algorithm) + Local Planner (DWA)
The navigation challenge involved testing Dijkstra's and D* algorithms as global planners and the Dynamic Window Approach (DWA) and Timed Elastic Band (TEB) as local planners. 
<br/><br/>
Five trials were conducted in Demo World Two for each algorithm to navigate from the initial deployment to the coordinate (5.0, 0.0) using RTABMap for the real-time map source. 
Although these trials may not thoroughly test each algorithm to their optimal performance, they provided the best estimate for evaluating our specific use case. 
The global planners were tested in conjunction with the TEB local planner, while the local planners were tested using Dijkstra's algorithm as the global planner. 
All algorithms were set to have a goal tolerance of 0.3 in yaw and 0.15 in xy direction to prevent unnecessary infinite adjustments from slight drifts.
Two main results were recorded for each test: the final global coordinate reached and whether the robot rotated 360 degrees more than five times without movement, indicating it was stuck. 
These results were averaged for each algorithm to assess performance. 
Although the D* algorithm was not tested in simulation, the combination of Dijkstra with the TEB algorithm achieved a 100% success rate for the simple navigation task attempted.
<br/><br/>
The test results in Appendix B show that the TEB algorithm is superior, with a 100% success rate compared to DWA's 40% and a lower task completion time of 38.131 seconds
in simulation time compared to DWA's 58.482 seconds when tested under the same conditions in demo world two with the goal object at (6.0, -2.0). 
However, a trend was observed where the TEB algorithm did not consider local obstacles when renewing goals, sometimes driving into obstacles. 
This unacceptable behaviour occured mainly when the robot renews the goal right next to an obstacle, making it invisible to the attached camera due to the unique shape of the obstacle (e.g. the base of a construction cone). 
This limitation arises from using a single perception sensor fixed at a certain height, effectively preventing a full awareness of immediate surroundings. 
As the project strictly requires the robot not to collide with any obstacles, the DWA algorithm better suits this project despite its lower performance. 
<br/><br/>

### Search – Random Obstacle Avoidance vs Frontier-Based Exploration
The goal search aspect of this project can be further divided into two subcategories: goal detection and navigation. 
Goal detection was handled by a simple thresholding method, while the navigation part had two different algorithms of: Random Obstacle Avoidance (ROA) and Frontier-Based Exploration (FBE). 
<br/><br/>
Five trials were conducted in Demo World Two to search for a goal object at (14.0, 4.0) using both ROA and FBE algorithms.
In these trials, the TurtleBot localised using the RTABMap algorithm with an RGB-D camera, and navigation was performed using Dijkstra's algorithm as the global planner with DWA algorithm as the local planner. 
The success rate of the ROA algorithm was detrimental, being 0%, while the FBE algorithm was reliable, successfully locating the goal all the time.
<br/><br/>
The poor success rate of the ROA algorithm can be attributed to its simplicity. 
The algorithm instructed the robot to turn right whenever an obstacle was detected in the depth image from the RGB-D camera. 
While this method successfully avoided all upcoming obstacles, it was insufficient to handle the curved placement of construction cones and cardboard boxes, causing the robot to loop around the same area over and over again.
The only fix to this problem would be modifying the algorithm to appropriately decide which way to turn when blocked by an obstacle. 
However, even if this solution succeeds, the performance is expected to be worse than the FBE counterpart, as FBE scans and priortises all unknown spaces, while ROA solely focuses on avoiding immediate obstacles and does not consider duplicate paths.

