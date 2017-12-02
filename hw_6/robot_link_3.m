function robot = robot_link_3(mass, length, Inertia)
%MDL

clear L

%% Geometry
%%%%%%%%%%%%% theta, d, a, alpha, revolute or prismatic, offset
L(1) = Link([ 0     0     length          0    0        0], 'standard');
L(2) = Link([ 0 	0     length        0        0        0], 'standard');
L(3) = Link([ 0 	0      length        0        0        0], 'standard');

%% Dynamic Params

% link 1
L(1).m = mass;
% L(1).I = [Inertia 0 0; 0 Inertia 0; 0 0 Inertia];
L(1).I = [0 0 Inertia];
L(1).r = [-length/2 0 0];

% link 2
L(2).m = mass;
% L(2).I = [Inertia 0 0; 0 Inertia 0; 0 0 Inertia];
L(2).I = [0 0 Inertia];
L(2).r = [-length/2 0 0];

% link 3
L(3).m = mass;
% L(3).I = [Inertia 0 0; 0 Inertia 0; 0 0 Inertia];
L(3).I = [0 0 Inertia];
L(3).r = [-length/2 0 0];

% gear ratios
L(1).G = 1;
L(2).G = 1;
L(3).G = 1;

% motor inertia
L(1).Jm = 0;
L(2).Jm = 0;
L(3).Jm = 0;


%% defining the robot now
robot = SerialLink(L, 'name', '3DOF', ...
    'manufacturer', 'Killpack Inc.');

% some useful poses
% qz = [0 0 0 0]; % zero angles, L shaped pose
robot.gravity = [0, -9.81, 0];

clear L

end