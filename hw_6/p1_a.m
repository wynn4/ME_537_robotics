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

tau = recursive_newton_euler(rrr, q, qdot, qddot);


%% part b
% calculate torques using the toolbox's rne and compare

tau_toolbox = rrr.rne(q, qdot, qddot);

% output
disp("tau vs toolbox tau:")
disp(tau)
disp(tau_toolbox)

% the result is the same!
% tau =
% 
%    -5.5155    1.5871    1.4951
% tau_toolbox =
% 
%    -5.5155    1.5871    1.4951

%% part c
% use rne function to get Inertai (M) matrix

M1 = recursive_newton_euler(rrr, q, [0 0 0], [1 0 0]);
M2 = recursive_newton_euler(rrr, q, [0 0 0], [0 1 0]);
M3 = recursive_newton_euler(rrr, q, [0 0 0], [0 0 1]);

M = [M1', M2', M3'];

M_toolbox = rrr.inertia(q);

% not quite right.  I think there's a problem with what I'm passing
% M =
% 
%    -4.4668   -5.0065   -5.4428
%     1.9302    1.7605    1.4939
%     1.4939    1.4939    1.4373

% M_toolbox =
% 
%     1.0825    0.5428    0.1066
%     0.5428    0.3731    0.1066
%     0.1066    0.1066    0.0500


%% part d
% use rne function to get Coriolis and compare to toolbox

C = recursive_newton_euler(rrr, q, qdot, [0 0 0]);

C_toolbox = rrr.coriolis(q, qdot);

% this one is really off for some reason...
% C =
% 
%    -5.5729    1.4248    1.4132

% C_toolbox =
% 
%     0.0531   -0.0776   -0.1073
%     0.0715   -0.0592   -0.0444
%     0.0271   -0.0148         0