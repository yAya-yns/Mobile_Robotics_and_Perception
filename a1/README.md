Main file: `ROB521_assignment1.m`
Code available: https://github.com/yAya-yns/Mobile_Robotics_and_Perception/

Credit: S L Waslander

## Introduction: ##

This project introduces the idea of motion planning for holonomic robots that can move in any direction and change direction of motion instantaneously.  Although unrealistic, it can work quite well for complex large scale planning. You will generate mazes to plan through and employ the PRM algorithm presented in lecture as well as any variations you can invent in the later sections.

There are three objectives to complete:
- objective 1: implement the PRM algorithm to construct a graph connecting start to finish nodes.
- objective 2: find the shortest path over the graph by implementing the Dijkstra's or A* algorithm.
- objective 3: identify sampling, connection or collision checking strategies that can reduce runtime for mazes.

## Objective 1: ##
Random points generator is applied for objective 1 taks, shown below:
![Alt text](/a1/assignment1_q1.png "Random Generated point linked with k-nearest point, k=8")

For objective 1: 
- 500 points are generated initially.
- Points too closed to the wall (<0.1 unit) are removed. 
- k-nearest search are performed. 
- To balance the speed and performance, k=8 are selected.
- path crossing the wall are removed.

Observation: 
- Many points/paths are wasted due to the location they generated. However, some "lucky" points allowed us to quickly cross the grids straight.


## Objective 2: ##
Random points generator is applied for objective 1 taks, shown below:
![Alt text](/a1/assignment1_q2.png "A-Star Algorithm")

For objective 2: 
- A-Star algorithm are applied
- The heuristics is the Euclidean distance to the goal. 
- I also set the weight of the heuristics. I found actual cost:heuristics = 1:1 is good enough. 

Observation: 
- As shown, the probablistic generated map allow us to cross the map in the most efficient manner. However, the computational cost are quite high. The detailed will be comparied 

