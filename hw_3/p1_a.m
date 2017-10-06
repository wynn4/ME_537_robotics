% Jesse Wynn
% Robotics HW 3

clc
clear

% Problem 7-2 from the book

% experiment with the teach method with the puma560 robot

q = [0, 0, 0, 0, 0, 0];

mdl_puma560

figure(1)
p560.plot(q)

p560.teach

