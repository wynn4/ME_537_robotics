clc
clear
close all

% load baxter
mdl_baxter

% the four q angle sets we used
q1 = [-1.60147594 -1.30196619 -0.95183508  0.23086411 -3.04571885 -0.4153253 3.03268002];
q2 = [-0.21015537 -1.09871374 -1.04847587  1.19995647 -3.04686934 -0.64197096 3.03498099];
q3 = [ 0.3831117  -0.59518455 -0.79460205  1.09794675 -2.02562163 -0.76430593 3.03574798];
q4 = [ 1.05998072 -0.25272333 -0.64043698  0.77887875  1.03045159  0.39001462 3.03536448];
q = [q1; q2; q3; q4];

% figure(1), clf
% 
% right.plot(q1)
% right.plot(q2)
% right.plot(q3)
% right.plot(q4)

% estimated robot poses
T1 = right.fkine(q1)
T2 = right.fkine(q2)
T3 = right.fkine(q3)
T4 = right.fkine(q4)



