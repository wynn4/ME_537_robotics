% Problem 2 part f
% Joint Torques given end-effector forces

clc
clear

% joint lengths
a1 = 1;
a2 = 1;

% joint angles
q = [0 0];

% EE force
F = [1 0 0]';

% wrench
Q = [F; [0 0 0]']; % [F_x; Tau_x] (assuming Tau_x = 0 0 0 since none given??

% Jacobian for 2 link robot
J = [-a2*sin(q(1))*cos(q(2)) - a1*sin(q(1)), -a2*sin(q(2))*cos(q(1));
     a2*cos(q(1))*cos(q(2)) + a1*cos(q(1)), -a2*sin(q(2))*sin(q(1));
     0, -a2*sin(q(1))*sin(q(1))*cos(q(2)) - a2*cos(q(1))*cos(q(1))*cos(q(2));
     0, -sin(q(1));
     0, cos(q(1));
     1, 0];

% compute joint torques
Tau_joints = J'*Q