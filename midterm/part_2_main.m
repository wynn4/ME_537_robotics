% Main script for Midterm Part 2

% ME 537 Robotics
% Jesse Wynn

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%_____________________________READ_ME_____________________________%%%%
%%%%_________________________________________________________________%%%%
%%%% This script defines 3 robots.  The first is the full robot, the %%%%
%%%% second is the first robot thru the third joint, and the third   %%%%
%%%% robot is the first robot thru the second joint. These two extra %%%%
%%%% robots allow me to get the position of the robot at a few key   %%%%
%%%% locations using forward kinematics.  Once the robot's initial   %%%%
%%%% joint angles, goal location, and obstacle location have been    %%%%
%%%% defined, I call the function compute_robot_path(). This func-   %%%%
%%%% tion does a few things. First, it uses the Jacobian Transpose   %%%%
%%%% method to compute an inverse kinematics solution for the robot  %%%%
%%%% arm to reach the goal location. Once this final joint-angle     %%%%
%%%% configuration is known, the function chooses a collision-avoid  %%%%
%%%% ance path. This path can take two forms: (1) Rotate about q1 to %%%%
%%%% turn the robot away from the obstacle, get all other joints in- %%%%
%%%% to final configuration, and then rotate q1 into final config.   %%%%
%%%% (2) Rotate about q2 to lift or drop the robot away from the ob- %%%%
%%%% stacle, tuck in the end-effector, rotate all other joints into  %%%% 
%%%% final config, and finally rotate q2 into final config and un-   %%%%
%%%% tuck the end-effector. The collision-avoidance path is chosen   %%%%
%%%% by 'feeling out' the environment using forward kinematics with  %%%%
%%%% the three robots. This method works for many but not all obst-  %%%%
%%%% acle, arm, and goal configurations.  This method also ignores   %%%%
%%%% joint limits, and collision with the robot's own frame. In some %%%%
%%%% cases the robot also completes redundant joint revolutions      %%%%
%%%% before reaching it's final configuration. Once everything has   %%%%
%%%% been computed, the robot is animated to follow the path, and a  %%%%
%%%% second figure displays the position history of the end-effector %%%%
%%%% joint 4, joint 3, the robot in final config, and the obstacle.  %%%%
%%%%_________________________________________________________________%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear
close all

% define the robot
robot = robot_4dof();   % 4 dof for now...
robot3 = robot_3dof();
robot2 = robot_2dof();

% initial q angle
% q_init = [0 degtorad(-60) 0 degtorad(60)];
q_init = [0 degtorad(-60) degtorad(0) degtorad(60)];

% obstacle location info
obst_location = [0, 3, 2];    % [x, y, z] (meters)
% obst_location = [3, 0, 0];    % [x, y, z] (meters)
r = 1;  % radius (m)

% goal location info
goal = [0, 2, 4];   % [x, y, z] (meters)
% goal = [2, 0, 4];   % [x, y, z] (meters)


% for plot
[x,y,z] = sphere(30);

figure(1), clf
robot.plot(q_init)
hold on
surf(x+obst_location(1), y+obst_location(2), z+obst_location(3))
plot3(goal(1), goal(2), goal(3),'*')
view(110,20)

q_s = compute_robot_path(robot, robot3, robot2 , q_init, goal, obst_location, r);

% animate the robot
disp('moving robot...')
robot.animate(q_s)
disp('done')

%% save off some data for plotting
ee_positions = zeros(length(q_s),3);
jt4_positions = zeros(length(q_s),3);
jt3_positions = zeros(length(q_s),3);

for i = 1:length(q_s)
    % compute forward kinematics
    Tee = robot.fkine(q_s(i,:)).T;
    pos_ee = Tee(1:3,4)';
    Tjt4 = robot3.fkine(q_s(i,1:3)).T;
    pos_jt4 = Tjt4(1:3,4)';
    Tjt3 = robot2.fkine(q_s(i,1:2)).T;
    pos_jt3 = Tjt3(1:3,4)';
    
    % fill out the plotting vectors
    ee_positions(i,:) = pos_ee;
    jt4_positions(i,:) = pos_jt4;
    jt3_positions(i,:) = pos_jt3;
    
end

figure(2), clf
% plot the EE, joint4, and joint3 locations
plot3(ee_positions(:,1), ee_positions(:,2), ee_positions(:,3), '*r')
hold on
plot3(jt4_positions(:,1), jt4_positions(:,2), jt4_positions(:,3), '*g')
hold on
plot3(jt3_positions(:,1), jt3_positions(:,2), jt3_positions(:,3), '*m')
legend('end-effector','joint 4','joint 3')
surf(x+obst_location(1), y+obst_location(2), z+obst_location(3))
robot.plot(q_s(length(q_s),:))