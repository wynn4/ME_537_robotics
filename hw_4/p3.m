% Jesse Wynn
% Homework 4
% Problem 3

clc
close all
clear

% load in the 10 random poses
load('random_poses.mat')

% load in the corresponding joint angles
load('q_angles.mat')

% load the robot
robot = robot_6dof();

% two different starting sets of joint angles
q1 = [0.1 0.1 0.1 0.1 0.1 0.1];
q2 = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2];

q_initials = [q1; q2];

rows = size(q_initials);
rows = rows(1);

cur_pos_vec = zeros(3,1);

% IK solution finding methods

% error threshold
error_thresh = 0.01;  % 1mm

% time step
dt = 0.001;

% xidot (tuning param)
xidot = eye(6);

% T_des
T_des = robot.fkine(q_angles(1,:));
x = T_des.t(1);
y = T_des.t(2);
z = T_des.t(3);
des_pos_vec = [x; y; z];

% method 1: Damped Pseudo-Inverse

error = 100;
count = 0;

% for the two starting joint configurations:
% for i = 1:1
%     
%     % get the initial joint config
%     q_current = q_initials(1,:);
%     
%     % and while the EE pose error is too large
%     while error > error_thresh
%         
%         % compute the Jacobian J(q_current)
%         J = robot.jacob0(q_current);
%         
%         % figure out where the arm is
%         T_cur = robot.fkine(q_current);
%         x_pos = T_cur.t(1);
%         y_pos = T_cur.t(2);
%         z_pos = T_cur.t(3);
%         
%         cur_pos_vec = [x_pos; y_pos; z_pos];
%         
%         % compute the error
%         error = norm(cur_pos_vec - des_pos_vec)
%         
%         
%         % compute qdot
%         qdot = inv(J) * xidot;
%         
%         % move toward the desired pose
%         q_next = q_current + (qdot .* dt);
%         
%         % reset
%         q_current = diag(q_next)';
%        
%         
%         count = count + 1;
%         disp(count)
%         
%         pause(0.01)
%     end
% end
