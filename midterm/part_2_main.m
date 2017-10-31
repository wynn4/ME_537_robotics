% Main script for Midterm Part 2

% ME 537 Robotics
% Jesse Wynn

clc
clear
close all

% define the robot
robot = robot_4dof();   % 4 dof for now...
robot3 = robot_3dof();
robot2 = robot_2dof();
robot1 = robot_1dof();

% initial q angle
% q_init = [0 degtorad(-60) 0 degtorad(60)];
q_init = [0 degtorad(-60) degtorad(0) degtorad(60)];

% obstacle location info
obst_location = [0, 3, 2];    % [x, y, z] (meters)
r = 1;  % radius (m)

% goal location info
goal = [0, 2, 4];   % [x, y, z] (meters)

% for plot
[x,y,z] = sphere(30);

figure(1), clf
robot.plot(q_init)
hold on
surf(x+obst_location(1), y+obst_location(2), z+obst_location(3))
plot3(goal(1), goal(2), goal(3),'*')
view(110,20)

q_s = compute_robot_path(robot, q_init, goal, obst_location, r);

% animate the robot
disp('moving robot...')
robot.animate(q_s)
disp('done')
