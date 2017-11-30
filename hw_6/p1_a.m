clc
clear
close all

%% part a
% params of all 3 links
m = 1;      % link mass kg
l = 0.4;    % link lenght (m)
Izz = 0.01; % link rotational inertia

% load 3 link revolute robot
rrr = robot_link_3(m, l, Izz);
clc

% q qdot qddot desired
q = [pi/4 pi/4 pi/4];
qdot = [pi/6 -pi/4 pi/3];
qddot = [-pi/6 pi/3 pi/6];

% solve for required joint torques given q, qdot, and qddot using
% recursive newton-euler method

[f, tau] = recursive_newton_euler(rrr, q, qdot, qddot);


%% part b
% calculate torques using the toolbox's rne and compare

tau_toolbox = rrr.rne(q, qdot, qddot);

% output
disp("tau vs toolbox tau:")
disp(tau(3,:))
disp(tau_toolbox)

% I don't get the same result but I do get the same result as the Dr.
% Killpack

%% part c


%% part d




