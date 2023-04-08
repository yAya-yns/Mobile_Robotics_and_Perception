% =========
% Task_3_part1.m
% =========
%
% This assignment will introduce you to the idea of first building an
% occupancy grid then using that grid to estimate a robot's motion using a
% particle filter.
% 
% There are two questions to complete (5 marks each):
%
%    Question 1: code occupancy mapping algorithm 
%    Question 2: see main_part2.m
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plot/movie, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file
% and the two resulting AVI files from Questions 1 and 2.
%
% requires: basic Matlab, 'gazebo.mat'
%
% Steven Waslander, April 2023
%
clear all;

% set random seed for repeatability
rng(1);

% ==========================
% load the dataset from file
% ==========================
%
%    ground truth poses: t_true x_true y_true theta_true
%    odometry measurements: t_odom v_odom omega_odom
%    laser scans: t_laser y_laser
%    laser range limits: r_min_laser r_max_laser
%    laser angle limits: phi_min_laser phi_max_laser
%
load gazebo.mat;

% =======================================
% Question 1: build an occupancy grid map
% =======================================
%
% Write an occupancy grid mapping algorithm that builds the map from the
% perfect ground-truth localization.  Some of the setup is done for you
% below.  The resulting map should look like "q1_soln.png".  You can
% watch the movie "q1_soln.mp4" to see what the enTe mapping process
% should look like.  At the end you will save your occupancy grid map to
% the file "occmap.mat" for use in Question 2 of this assignment.

% allocate a big 2D array for the occupancy grid
ogres = 0.05;                   % resolution of occ grid
ogxmin = -7;                    % minimum x value
ogxmax = 8;                     % maximum x value
ogymin = -3;                    % minimum y value
ogymax = 6;                     % maximum y value
ognx = (ogxmax-ogxmin)/ogres;   % number of cells in x direction
ogny = (ogymax-ogymin)/ogres;   % number of cells in y direction
oglo = zeros(ogny,ognx);        % occupancy grid in log-odds format
ogp = zeros(ogny,ognx);         % occupancy grid in probability format

% precalculate some quantities
numodom = size(t_odom,1);
npoints = size(y_laser,2);
angles = linspace(phi_min_laser, phi_max_laser,npoints);
dx = ogres*cos(angles);
dy = ogres*sin(angles);

% interpolate the noise-free ground-truth at the laser timestamps
t_interp = linspace(t_true(1),t_true(numodom),numodom);
x_interp = interp1(t_interp,x_true,t_laser);
y_interp = interp1(t_interp,y_true,t_laser);
theta_interp = interp1(t_interp,theta_true,t_laser);
omega_interp = interp1(t_interp,omega_odom,t_laser);
  
% set up the plotting/movie recording
vid = VideoWriter('q1_video.avi');
open(vid);
figure(1);
clf;
pcolor(ogp);
colormap(1-gray);
shading('flat');
axis equal;
axis off;
M = getframe;
writeVideo(vid,M);


% loop over laser scans (every fifth)
for i=1:5:size(t_laser,1)
    
    % ------insert your occupancy grid mapping algorithm here------
    if (abs(omega_interp(i)) > 0.1)  % avoid rotation being too fast and affect the estimation
        continue
    end
    T = [
      cos(theta_interp(i)), -sin(theta_interp(i)), 0, x_interp(i) - 0.1 * cos(theta_interp(i));
      sin(theta_interp(i)),  cos(theta_interp(i)), 0, y_interp(i);
      0,                               0,                               1, 0;
      0,                               0,                                0, 1;
    ];
    unoccupied_point_idx = 1;
    occupied_point_idx = 1;
    unoccupied_points = zeros([4, 1]);
    occupied_points = zeros([4, 1]);
    for k=1:size(y_laser,2)
        if isnan(y_laser(i, k))
            continue
        end
        steps = round(y_laser(i, k)/0.05);
        for p=1:steps
            point_range = p*0.05;
            if p <= steps - 2
                unoccupied_points(:, unoccupied_point_idx) = [point_range * cos(angles(k)); point_range * sin(angles(k)); 0; 1];
                unoccupied_point_idx = unoccupied_point_idx + 1;
            else
                occupied_points(:, occupied_point_idx) = [point_range * cos(angles(k)); point_range * sin(angles(k)); 0; 1];
                occupied_point_idx = occupied_point_idx + 1;
            end
        end
    end
    unoccupied_points_inertial = T * unoccupied_points;
    occupied_points_inertial = T * occupied_points;
    unoccupied_points_coords = unoccupied_points_inertial + [
        ones(1, size(unoccupied_points_inertial, 2))*(7);
        ones(1, size(unoccupied_points_inertial, 2))*(3);
        zeros(1, size(unoccupied_points_inertial, 2));
        zeros(1, size(unoccupied_points_inertial, 2)); ];
    unoccupied_points_coords(1:2, :) = round(unoccupied_points_coords(1:2, :)/0.05);
    unocc_indeces = sub2ind(size(ogp), unoccupied_points_coords(2, :), unoccupied_points_coords(1, :));
    occupied_points_coords = occupied_points_inertial + [
        ones(1, size(occupied_points_inertial, 2))*(7);
        ones(1, size(occupied_points_inertial, 2))*(3);
        zeros(1, size(occupied_points_inertial, 2));
        zeros(1, size(occupied_points_inertial, 2)); ];
    occupied_points_coords(1:2, :) = round(occupied_points_coords(1:2, :)/0.05);
    occ_indeces = sub2ind(size(ogp), occupied_points_coords(2, :), occupied_points_coords(1, :));
    oglo(unocc_indeces) = ogp(unocc_indeces) - 10;
    oglo(occ_indeces) = ogp(occ_indeces) + 10;
    ogp = exp(oglo)./(1+exp(oglo));
    % ------end of your occupancy grid mapping algorithm-------

    % draw the map
    clf;
    pcolor(ogp);
    colormap(1-gray);
    shading('flat');
    axis equal;
    axis off;
    
    % draw the robot
    hold on;
    x = (x_interp(i)-ogxmin)/ogres;
    y = (y_interp(i)-ogymin)/ogres;
    th = theta_interp(i);
    r = 0.15/ogres;
    set(rectangle( 'Position', [x-r y-r 2*r 2*r], 'Curvature', [1 1]),'LineWidth',2,'FaceColor',[0.35 0.35 0.75]);
    set(plot([x x+r*cos(th)]', [y y+r*sin(th)]', 'k-'),'LineWidth',2);
    
    % save the video frame
    M = getframe;
    writeVideo(vid,M);
    
    pause(0.1);
    
end

close(vid);
print -dpng q1.png

save occmap.mat ogres ogxmin ogxmax ogymin ogymax ognx ogny oglo ogp;

