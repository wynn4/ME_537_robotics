clc
clear
close all

% params
m = 1;      % link mass kg
l = 0.4;    % link lenght (m)

% load robots that represent center of mass (com) of the 3 links
robot1com = robot_com_link_1();
robot2com = robot_com_link_2();
robot3com = robot_com_link_3();

% load robots to represent origin of each link
robot1 = robot_link_1();
robot2 = robot_link_2();
robot3 = robot_link_3();
clc
% let q's be fixed for now
q1 = 0.2;
q2 = 0.2;
q3 = 0.2;
q = [q1 q2 q3];

% Initialize jacobians
Jcom1 = zeros(6,3);
Jcom2 = zeros(6,3);
Jcom3 = zeros(6,3);

% compute the jacobians
Jcom1(:,1) = robot1com.jacob0(q1);
Jcom2(:,1:2) = robot2com.jacob0([q1 q2]);
Jcom3(:,1:3) = robot3com.jacob0([q1 q2 q3]);

% get FK to all the Link coms
T1 = robot1com.fkine(q1).T;
R1 = T1(1:3,1:3);

T2 = robot2com.fkine([q1 q2]).T;
R2 = T2(1:3,1:3);

T3 = robot3com.fkine([q1 q2 q3]).T;
R3 = T3(1:3,1:3);

% define the inertia matrix (assume links are thin rods)
Imat = (m*l*l/12)*eye(3);

% get rotational part of M(q)
% K_rot = 1/2 * qdot' * {M(q)_rot} * qdot
M_rot = Jcom1(4:6,:)' * R1 * Imat * R1 *  Jcom1(4:6,:) + ...
    Jcom2(4:6,:)' * R2 * Imat * R2 *  Jcom2(4:6,:) + ...
    Jcom3(4:6,:)' * R3 * Imat * R3 *  Jcom3(4:6,:);

% get translational part of M(q)
M_trans = m * Jcom1(1:3,:)' * Jcom1(1:3,:) + ...
    m * Jcom2(1:3,:)' * Jcom2(1:3,:) + ...
    m * Jcom3(1:3,:)' * Jcom3(1:3,:);

% total M (it's symetric)
M = M_rot + M_trans;

% here we will assume corriolis terms are negligable :) (time constraints
% here...)
C = zeros(3);

% potential energy

% get 
g = [0 9.81 0]';
P1 = m * g' * T1(1:3,4);
P2 = m * g' * T2(1:3,4);
P3 = m * g' * T3(1:3,4);

P = P1 + P2 + P3;

% Then differentiate
% dP/dq1 = g1
% dP/dq2 = g2
% dP/dq3 = g3
% G = [g1 g2 g3]'

% In the end we get
% M * [qddot1 qddot2 qddot3]' + C * [qdot1 qdot2 qdot3]' + G * [q1 q2 q3] =
% [Tau1 Tau2 Tau3]'





