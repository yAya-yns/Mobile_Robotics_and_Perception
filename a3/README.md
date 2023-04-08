Main file: `main.m`

Code available: https://github.com/yAya-yns/Mobile_Robotics_and_Perception/tree/main/a3

Credit: S L Waslander

## Introduction: ##
This project will introduce you to the idea of first building an occupancy grid then using that grid to estimate a robot's motion using a particle filter.

There are two objectives to complete:
  - Objective 1: Implement occupancy mapping algorithm
  - Objective 2: Implement particle filter to localize from known map



## Objective 1: ##
We implement an algorithm to estimate the pose of the robot throughout motion using the wheel odometry data (t_odom, v_odom, omega_odom) and assuming a differential-drive robot model. The estimations are saved as variables: (x_odom y_odom theta_odom) so that the comparison plots can be generated below: 
![Alt text](/a3/q1.png "TODO")

https://user-images.githubusercontent.com/33261653/230699003-812a5e55-e27f-466c-97c1-d3daa2a48f42.mp4

## Objective 2: ##
We deliberately add some noise to the linear and angular velocities to simulate what real wheel odometry is like. The same algorithm from Objective#1 is used here. However, we loops 100 times with different random noise. From picture below, we discover that the path deviate much more as iteration increase.
![Alt text](/a3/q2.png "TODO")

https://user-images.githubusercontent.com/33261653/230699014-6173fa02-e731-45f1-9c1d-deb69c082b76.mp4
