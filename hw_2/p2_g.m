% Jesse Wynn HW2 ME 537 Robotics
clc
clear all
close all
clear imports

% Problem 2(g) from the hw printout

import +ETS3.*

a1 = 0.5;
a2 = 0.5;
a3 = 0.5;
a4 = 0.25;
a5 = 0.25;
a6 = 0.5;

E3 = Rz('q1') * Tz(a1) * Ry('q2') * Tx(a2) * Ry('q3') * Tx(a3) * Rx('q4') * Tx(a4) * Rz('q5') * Tx(a5) * Rx('q6') * Tx(a6);