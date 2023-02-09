% ======
% ROB521_assignment1.m
% ======
%
% This assignment will introduce you to the idea of motion planning for  
% holonomic robots that can move in any direction and change direction of 
% motion instantaneously.  Although unrealistic, it can work quite well for
% complex large scale planning.  You will generate mazes to plan through 
% and employ the PRM algorithm presented in lecture as well as any 
% variations you can invent in the later sections.
% 
% There are three questions to complete (5 marks each):
%
%    Question 1: implement the PRM algorithm to construct a graph
%    connecting start to finish nodes.
%    Question 2: find the shortest path over the graph by implementing the
%    Dijkstra's or A* algorithm.
%    Question 3: identify sampling, connection or collision checking 
%    strategies that can reduce runtime for mazes.
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plots, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file.
%
% requires: basic Matlab, 
%
% S L Waslander, January 2022
%
clear; close all; clc;

% set random seed for repeatability if desired
rng(1,'twister');

% ==========================
% Maze Generation
% ==========================
%
% The maze function returns a map object with all of the edges in the maze.
% Each row of the map structure draws a single line of the maze.  The
% function returns the lines with coordinates [x1 y1 x2 y2].
% Bottom left corner of maze is [0.5 0.5], 
% Top right corner is [col+0.5 row+0.5]
%

row = 5; % Maze rows
col = 7; % Maze columns
map = maze(row,col); % Creates the maze
start = [0.5, 1.0]; % Start at the bottom left
finish = [col+0.5, row]; % Finish at the top right

h = figure(1);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

% ======================================================
% Question 1: construct a PRM connecting start and finish
% ======================================================
%
% Using 500 samples, construct a PRM graph whose milestones stay at least 
% 0.1 units away from all walls, using the MinDist2Edges function provided for 
% collision detection.  Use a nearest neighbour connection strategy and the 
% CheckCollision function provided for collision checking, and find an 
% appropriate number of connections to ensure a connection from  start to 
% finish with high probability.


% variables to store PRM components
nS = 500;  % number of samples to try for milestone creation
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]
distance_from_wall = 0.1;
disp("Time to create PRM graph")
tic;
% ------insert your PRM generation code here-------

b_l_corner = [0.5, 0.5]; % Bottom left corner of maze is [0.5 0.5], 
t_r_corner = [col+0.5, row+0.5]; % Top right corner is [col+0.5 row+0.5]
k = 8; % number of nearest neighbour

attempt_milestones_x = rand(nS, 1) * col + b_l_corner(1);
attempt_milestones_y = rand(nS, 1) * row + b_l_corner(2);
attempt_milestones = horzcat(attempt_milestones_x, attempt_milestones_y);

% intermediate results:
% milestones = vertcat(milestones, attempt_milestones)

% filter 1: removing points too closed to wall

for i = 1: length(attempt_milestones(:, 1))
    if MinDist2Edges([attempt_milestones(i, :)], map) >= distance_from_wall
        milestones = vertcat(milestones, attempt_milestones(i, :));
    end
end
disp("number of points after removing near wall's is: ");
disp(size(milestones));

% use milestones_map and edges_map 
milestones_map = horzcat(linspace(1, length(milestones(:, 1)), length(milestones(:, 1)))', milestones); % each row: [idx, ptX, ptY]
raw_edges_map = zeros(k * length(milestones(:, 1)), 7); % [startIdx, targetIdx, startPt_x, startPt_y, endPt_x, endPt_y, cost]
for i = 1: length(milestones_map(:, 1))
    milestones_map_wo_i = vertcat(milestones_map(1:i-1, :), milestones_map(i+1:end, :));  % avoiding self connection
    for j = 1:k
        nearest_idx_wo_i = dsearchn(milestones_map_wo_i(:, 2:3), milestones_map(i, 2:3));  % find nearest point in the searching space
        nearest_idx = milestones_map_wo_i(nearest_idx_wo_i, 1); % getting the real idx from complete milestones_map
        cost = norm(milestones_map(i, 2:3) - milestones_map(nearest_idx, 2:3));
        raw_edges_map((i-1)*k+j, :) = horzcat(i, nearest_idx, milestones_map(i, 2:3), milestones_map(nearest_idx, 2:3), cost);
        milestones_map_wo_i = vertcat(milestones_map_wo_i(1:nearest_idx_wo_i -1, :), milestones_map_wo_i(nearest_idx_wo_i+1:end, :));  % remove the nearest from searching space
    end
end


% % checking Collision.
edges_map = zeros(1, length(raw_edges_map(1, :)));
for i = 1: length(raw_edges_map(:, 1))
    ptA = raw_edges_map(i, 3:4);
    ptB = raw_edges_map(i, 5:6);
    [ inCollision, edge ] = CheckCollision( ptA, ptB, map );
    if inCollision ~= 1
        edges_map = vertcat(edges_map, raw_edges_map(i, :));
    end
end
edges_map = edges_map(2:end, :);

edges = edges_map(:, 3:6);


% ------end of your PRM generation code -------
toc;

figure(1);
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta') % line uses [x1 x2 y1 y2]
end
str = sprintf('Q1 - %d X %d Maze PRM', row, col);
title(str);
drawnow;

print -dpng assignment1_q1.png


% =================================================================
% Question 2: Find the shortest path over the PRM graph
% =================================================================
%
% Using an optimal graph search method (Dijkstra's or A*) , find the 
% shortest path across the graph generated.  Please code your own 
% implementation instead of using any built in functions.

disp('Time to find shortest path');
tic;

% Variable to store shortest path
spath = []; % shortest path, stored as a milestone row index sequence


% ------insert your shortest path finding algorithm here-------
startIdx = 1; 
finishIdx = 2;
hWeight = 1; % weight of heuristic. 
maxIter = 1000;

% building a graph data structure
G = graph(edges_map(:, 1)', edges_map(:, 2)', edges_map(:, 7)'); % startIdx, targetIdx, cost
G = graph(unique(G.Edges)); % removing duplicated edges
spath = aStar(G, startIdx, finishIdx, milestones_map, hWeight, maxIter);    

% ------end of shortest path finding algorithm------- 
toc;    

% plot the shortest path
figure(1);
for i=1:length(spath)-1
    plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
end
str = sprintf('Q2 - %d X %d Maze Shortest Path', row, col);
title(str);
drawnow;

print -dpng assingment1_q2.png


% ================================================================
% Question 3: find a faster way
% ================================================================
%
% Modify your milestone generation, edge connection, collision detection 
% and/or shortest path methods to reduce runtime.  What is the largest maze 
% for which you can find a shortest path from start to goal in under 20 
% seconds on your computer? (Anything larger than 40x40 will suffice for 
% full marks)


clear; close all; clc;

row = 40;
col = 40;
map = maze(row,col);
start = [0.5, 1.0];
finish = [col+0.5, row];
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]

h = figure(2);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

fprintf("Attempting large %d X %d maze... \n", row, col);
tic;        
% ------insert your optimized algorithm here------

% PART 1 %%%%%%%%%%%%%%%%%%%%%

b_l_corner = [0.5, 0.5]; % Bottom left corner of maze is [0.5 0.5], 
t_r_corner = [col+0.5, row+0.5]; % Top right corner is [col+0.5 row+0.5]

distance_from_wall = 0.1;

% k = 7; % number of nearest neighbour
% nS = 6500;
% attempt_milestones_x = rand(nS, 1) * col + b_l_corner(1);
% attempt_milestones_y = rand(nS, 1) * row + b_l_corner(2);
% attempt_milestones = horzcat(attempt_milestones_x', attempt_milestones_y');


k = 4; % number of nearest neighbour
density = 1;
offset = 0.5/density;
x_linspace = linspace(offset + b_l_corner(1), col+b_l_corner(1)-offset, col*density);
y_linspace = linspace(offset + b_l_corner(2), row+b_l_corner(2)-offset, row*density);

attempt_milestones = zeros(length(x_linspace)* length(y_linspace), 2);
count = 1;
for i = 1: length(x_linspace)
    x = x_linspace(i);
    for j = 1:length(y_linspace)
        y = y_linspace(j);
        attempt_milestones(count, :) = [x, y];
        count = count+1;
    end
end


% intermediate results:
% milestones = vertcat(milestones, attempt_milestones)

% filter 1: removing points too closed to wall

for i = 1: length(attempt_milestones(:, 1))
    if MinDist2Edges([attempt_milestones(i, :)], map) >= distance_from_wall
        milestones = vertcat(milestones, attempt_milestones(i, :));
    end
end
disp("number of points after removing near wall's is: ");
disp(size(milestones));

% use milestones_map and edges_map 
milestones_map = horzcat(linspace(1, length(milestones(:, 1)), length(milestones(:, 1)))', milestones); % each row: [idx, ptX, ptY]
raw_edges_map = zeros(k * length(milestones(:, 1)), 7); % [startIdx, targetIdx, startPt_x, startPt_y, endPt_x, endPt_y, cost]
for i = 1: length(milestones_map(:, 1))
    milestones_map_wo_i = vertcat(milestones_map(1:i-1, :), milestones_map(i+1:end, :));  % avoiding self connection
    for j = 1:k
        nearest_idx_wo_i = dsearchn(milestones_map_wo_i(:, 2:3), milestones_map(i, 2:3));  % find nearest point in the searching space
        nearest_idx = milestones_map_wo_i(nearest_idx_wo_i, 1); % getting the real idx from complete milestones_map
        cost = norm(milestones_map(i, 2:3) - milestones_map(nearest_idx, 2:3));
        raw_edges_map((i-1)*k+j, :) = horzcat(i, nearest_idx, milestones_map(i, 2:3), milestones_map(nearest_idx, 2:3), cost);
        milestones_map_wo_i = vertcat(milestones_map_wo_i(1:nearest_idx_wo_i -1, :), milestones_map_wo_i(nearest_idx_wo_i+1:end, :));  % remove the nearest from searching space
    end
end

% % checking Collision.
edges_map = zeros(1, length(raw_edges_map(1, :)));
for i = 1: length(raw_edges_map(:, 1))
    ptA = raw_edges_map(i, 3:4);
    ptB = raw_edges_map(i, 5:6);
    [ inCollision, edge ] = CheckCollision( ptA, ptB, map );
    if inCollision ~= 1
        edges_map = vertcat(edges_map, raw_edges_map(i, :));
    end
end
edges_map = edges_map(2:end, :);
edges = edges_map(:, 3:6);

figure(2);
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta') % line uses [x1 x2 y1 y2]
end
str = sprintf('Q3 - %d X %d Maze PRM', row, col);
title(str);
drawnow;
print -dpng assignment1_q3_map.png
disp("map generated");


% PART 2 %%%%%%%%%%%%%%%%%%%%%
spath=[];
startIdx = 1; 
finishIdx = 2;
hWeight = 1; % weight of heuristic. 
maxIter = 7000;

% building a graph data structure
G = graph(edges_map(:, 1)', edges_map(:, 2)', edges_map(:, 7)'); % startIdx, targetIdx, cost
G = graph(unique(G.Edges)); % removing duplicated edges
disp('finding Path');
spath = aStar(G, startIdx, finishIdx, milestones_map, hWeight, maxIter);    
if isempty(spath)
    return
end


% ------end of your optimized algorithm-------
dt = toc;

figure(2); hold on;
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta')
end
if (~isempty(spath))
    for i=1:length(spath)-1
        plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
    end
end
str = sprintf('Q3 - %d X %d Maze solved in %f seconds', row, col, dt);
title(str);

print -dpng assignment1_q3.png

