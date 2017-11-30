function [f, tau] = recursive_newton_euler(robot, q, qdot, qddot)

% implement Recursive Newton-Euler method

% Syntax convention Rotation R_from frame _ to frame
% R_from_to
% T_from_to

% link length
l = robot.links(1,1).a;

% link mass
m = robot.links(1,1).m;

% get transformations and rotations
T_1_0 = robot.A(1,q).T;
R_1_0 = T_1_0(1:3, 1:3);

T_2_1 = robot.A(2,q).T;
R_2_1 = T_2_1(1:3,1:3);

T_3_2 = robot.A(3,q).T;
R_3_2 = T_3_2(1:3,1:3);

R_0_1 = R_1_0';
R_1_2 = R_2_1';
R_2_3 = R_3_2';

R_0_2 = R_1_2 * R_0_1;
R_0_3 = R_2_3 * R_1_2 * R_0_1;

% initial conditons
w0 = [0 0 0]';
alpha0 = [0 0 0]';
ae0 = [0 0 0]';

z = [0 0 1]';

% gravity
g = [0 9.81 0]';

% inertia matrix
% I = Izz * eye(3);
% in our case, all links have the same inertia properties
I = robot.links(1,1).I;

% first, find the a's, alpha's, w's starting at link 1

% w's
w1 = R_0_1 * w0 + R_0_1 * z * qdot(1);  % link 1
w2 = R_1_2 * w1 + R_0_2 * z * qdot(2);  % link 2
w3 = R_2_3 * w2 + R_0_3 * z * qdot(3);  % link 3

% alpha's
alpha1 = R_0_1 * alpha0 + R_0_1 * z * qddot(1) + cross(w1, R_0_1 * z * qdot(1));
alpha2 = R_1_2 * alpha1 + R_0_2 * z * qddot(2) + cross(w2, R_0_2 * z * qdot(2));
alpha3 = R_2_3 * alpha2 + R_0_3 * z * qddot(3) + cross(w3, R_0_3 * z * qdot(3));

% r_0_1 is distance from joint 0 to joint 1
r_0_1 = [l 0 0]';   % x y z
r_1_2 = [l 0 0]';
r_2_3 = [l 0 0]';

r_0_c = r_0_1/2;
r_1_c = r_1_2/2;
r_2_c = r_2_3/2;

r_plus_c = -[l; 0; 0]/2;    % not sure about this one...

% ae's
ae1 = R_0_1 * ae0 + cross(alpha1, r_0_1) + cross(w1, cross(w1, r_0_1));
ae2 = R_1_2 * ae1 + cross(alpha2, r_1_2) + cross(w2, cross(w2, r_1_2));
ae3 = R_2_3 * ae2 + cross(alpha3, r_2_3) + cross(w3, cross(w3, r_2_3));

% ac's
ac1 = R_0_1 * ae0 + cross(alpha1, r_0_c) + cross(w1, cross(w1, r_0_c));
ac2 = R_1_2 * ae1 + cross(alpha2, r_1_c) + cross(w2, cross(w2, r_1_c));
ac3 = R_2_3 * ae2 + cross(alpha3, r_2_c) + cross(w3, cross(w3, r_2_c));

% now go backwards from the end-effector to the base frame to get f and Tau
Tau4 = [0 0 0]';    % no torques on ee
f4 = [0 0 0]';      % no forces on ee

R_4_3 = eye(3); % link 3 and end effector (frame 4) share the same frame

% compute f's
f3 = R_4_3 * f4 + m * ac3 - m * R_0_3 * g;
f2 = R_3_2 * f3 + m * ac2 - m * R_0_2 * g;
f1 = R_2_1 * f2 + m * ac1 - m * R_0_1 * g;

% compute Tau's
Tau3 = R_4_3 * Tau4 - cross(f3, r_2_c) + cross(R_4_3 * f4, r_plus_c) + I * alpha3 + cross(w3, I * w3);
Tau2 = R_3_2 * Tau3 - cross(f2, r_1_c) + cross(R_3_2 * f3, r_plus_c) + I * alpha2 + cross(w2, I * w2);
Tau1 = R_2_1 * Tau2 - cross(f1, r_0_c) + cross(R_2_1 * f2, r_plus_c) + I * alpha1 + cross(w1, I * w1);

tau = [Tau1, Tau2, Tau3];

f = [f1, f2, f3];

end