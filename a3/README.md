Main file: `main.m`

Code & Demo available at: https://github.com/yAya-yns/Mobile_Robotics_and_Perception/tree/main/a3

Credit: S L Waslander

## Introduction: ##
This project will introduce you to the idea of first building an occupancy grid then using that grid to estimate a robot's motion using a particle filter.

There are two objectives to complete:
  - Objective 1: Implement occupancy mapping algorithm
  - Objective 2: Implement particle filter to localize from known map



## Objective 1: ##
In this section, the process of constructing an occupancy grid using a mobile robot is discussed. The robot provides three pieces of information: laser scan data, its location in an inertial frame, and its orientation. A simple algorithm is employed to accomplish the task of constructing the grid. At each time step, a transformation matrix is created to translate points from the robot's frame into the inertial frame. The laser scan is then iterated over, and each point at the end of the laser is identified as an obstacle. The points are then translated into coordinates, and the probability that a particular map entry is occupied is increased. Each subsequent point before the end of the laser is also translated into coordinates, and the probability that those particular squares are unoccupied is increased. Once all lasers have been iterated over, the map is thresholded, with squares that are more likely to be occupied marked as such, and those more likely to be unoccupied marked as unoccupied. Figure 1 below shows the resulting occupancy grid.

![Alt text](/a3/q1.gif "occupancy mapping process")
![Alt text](/a3/q1.png "produced map")


## Objective 2: ##
In this study, due to the computational complexity involved in implementing a particle filter, only the outermost sensor readings were considered. These readings were then compared with the expected sensor readings based on the positions of each point. To update the weight associated with each particle, the euclidean distance between the expected and actual laser measurements was calculated. The weights were then updated assuming a Gaussian distribution. To assess the performance of the particle filter, the weighted average of all the particles was computed. This allowed for an evaluation of the filter's effectiveness, and a comparison was made between the errors produced by the weighted average and those produced by an odometry-based method. Figure 2 below provides a graphical representation of these errors.

![Alt text](/a3/q2.gif "particle filter localization process")
![Alt text](/a3/q2.png "Error comparison between particle filter and visual odom")

