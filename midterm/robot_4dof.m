function robot = robot_4dof()
%MDL

clear L

%%%%%%%%%%%%% theta, d, a, alpha, revolute or prismatic, offset
L(1) = Link([ 0     4.0     0          pi/2    0        pi/2], 'standard');
L(2) = Link([ 0 	0.0      2.0        0        0        0], 'standard');
L(3) = Link([ 0 	0.0      2.0        0        0        0], 'standard');
L(4) = Link([ 0     0.0      1.75         0    0         0], 'standard');
% L(5) = Link([ 0     0.0     1.75        pi/2    0         0], 'standard');



%% defining the robot now
robot = SerialLink(L, 'name', '4DOF', ...
    'manufacturer', 'Killpack Inc.');

% some useful poses
qz = [0 0 0 0]; % zero angles, L shaped pose

clear L

end


% L(1) = Link([ 0     4.0     0          pi/2    0        pi/2], 'standard');
% L(2) = Link([ 0 	0.0      0.0        pi/2        0        pi/2], 'standard');
% L(3) = Link([ 0     4.0      0         0    0         pi/2], 'standard');
% % L(4) = Link([ 0     0.0     1.75        pi/2    0         0], 'standard');