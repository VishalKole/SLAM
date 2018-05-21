# Simultaneous Localization and Mapping

Figure 1 shows the flow of nodes for the project. After waking up, the robot safe wanders without running into obstacles until localization. Once localized, it published global pose to the robot. This global pose is used by A* algorithm to find an optimal path with way-points to the destination. This path is then executed by the robot with obstacle avoidance for any obstacles that are not integrated into the map.

![image removed](https://github.com/VishalKole/SLAM/blob/master/flowpath.PNG)

### Localization
#### (a) Algorithm

We implemented the particle filter algorithm to solve the localization problem with the given map, Figure 2. Every particle has parameter Pose() and weight. The Pose() contains global x and y coordinates and orientation value. First, we initialized 16 × 16 locations, Figure 3, with 5 different orientations, Figure 4, around each possible starting points. Hence, we have 16 × 16 × 5 × 6 = 7680 initial particles with weight = 0:5. Second, we have a predict function to compute the difference between previous odometry data and current odometry data from the robot and apply the difference to all particles. Third, we modify the weight of each particle base on the corresponding map reading open distance values. We compute the distance to the wall on the map by using the Euclidean distance metric form the current pixel to the wall pixel, only for specified angles of the robot. The map reading function can return the left, right and front open distance values
base on the map given particle global location and orientation. We compare the return value with sonar and laser sensor data.


![image removed](https://github.com/VishalKole/SLAM/blob/master/particles.PNG)

 If the difference is under a threshold, we multiply 1 he weight with percent-change, otherwise, we divide the weight with percent-change. Fourth, if the number of particles is lower than half of the original size, we perform the re-sampling function. The re-sampling function sorts the particles by weight and randomly generates particle from the heaviest weight with increasing/decreasing x or y coordinates value between 0.1 to 0.2, and randomly choose orientation difference among -0.1, 0, or 0.1 radians. The generating loop continues until the number of particles equals to the initial number of particles. Fifth, the robot will safely wander to explore the environment and provide more space information to modify weights of particles to help localization. Sixth, the converge function will check if all particles converge to a close area under a certain threshold. If particles have converged, the mean Pose() value of all particles’ will be treated as a robot’s location and orientation.
 
#### (b) Simulation
**Starting location 1**
```
Video: https://youtu.be/eBJFBz7LyWc
```
Particles from location 2, 3, 4, and 5 were eliminated because of low probabilities, only
particles from location 1 and 6 remained at the beginning after few seconds. Both of
them were in the same hallway where the side distance was the same. They were able to
be distinguished until one cluster reaches the T-intersection. It converged at point (8.49,
11.54, 1.51) in 152 seconds.

**Starting location 2**
```
Video: https://youtu.be/JuIq3j97OGI
```
It is easy to get confused between starting location 2 and 3. They both have a similar
distance of hallway and front open space. But, once the robot started passing over the
pillar and made a turn, particles started converging. This simulation converged at point
(-15.91, 12.4, 3.38) in 64 seconds.


### Path planning
#### (a) Algorithm
Once the localization is done, it publishes the pose of the converging point to ’/localization’ topic. The path planning program, safegoto.py, subscribes to the ’/localization’
topic. Once a location is published, it will start to get the shortest path to destinations.
The main idea of our path planning is based on the A* search algorithm. The valid steps
are based on the given map’s pixels (open cells) with a safe distance (14 pixels) away
from occupied cells. We check this for all the valid pixels in the map and it gives us
a safe path(not too close to the wall) to the destination. We have used the Euclidean
distance as the heuristic distance and it works well for our application. The program will
then generate a valid path(these would be the pixels in the map). It will then convert
the pixels into global coordinates and deletes 19 consecutive pixels and keeps the 20th
3one to create way-points. The next step is to transform way-points from global coordinate to local coordinate(the local coordinates are the robot coordinates which change for
each starting point). Then, it performs safe-goto toward the destination based on the
way-points.

#### (b) Simulation
```
Video: https://drive.google.com/open?id=1KGq6OPEN0yz9VdlACjPwNIcJUa18Iesx
Video: https://drive.google.com/open?id=1W6vOTzQzdhDQE_wtl0Q1CXhJWC81bFwr
Video: https://drive.google.com/open?id=14PPVv0Z8l9ejWOXFiN1eNIJBR9maS8So
```
The above videos are the A-star path planning algorithm results for different points in
the map. We are showing all the possible points it consider but it selects only the shortest
and valid points for final way-point generation.
Once localization converges, the GUI is closed and localization node is shut down.
The A* star node uses this GUI to print path on the map. Figure 5 shows the path
between two points in the map.

<img src="https://github.com/VishalKole/SLAM/blob/master/astar%20path.PNG" height="400" width="400">

Initial implementation of GUI takes the complete path from A* star node and plots
it on the GUI. This did not help when there was no path with the specified conditions.
Hence, this was modified so that the path is represented on GUI as nodes are being added
to heap of A* implementation. If the distance of destination from walls is less than safe
distance or the destination is too close to the walls, there is no optimal path. In these
cases, the path planner tries to go forward and backward along the initial path.

### Software testing and observations
We tested the project on Gazebo before testing it on hardware and below are the links
to the video of end to end execution of the project.
```
Video: https://drive.google.com/open?id=1Cqg76KO6twYS3xS2BhMwG9scApS2M3H5
Video: https://drive.google.com/open?id=1JQ8aDrVjVeq5z5lFrxvfguLNQk64CouL
```

### Hardware testing results

#### Starting location 2
The localization also successfully converged to Clark’s global location (-18.2, 12.08, 3.24)
in 84 seconds

![](https://github.com/VishalKole/SLAM/blob/master/HW%20test.PNG)
![](https://github.com/VishalKole/SLAM/blob/master/HW%20test-loc1.PNG)
