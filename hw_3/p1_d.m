% Jesse Wynn
% Robotics HW 3

clc
% clear

% Problem 8-2 from the book

% for the puma560 robot, can you devise an situation where three axes are
% parallel?

mdl_puma560
bot = p560;

q = [0 0 0 0 0 0];
bot.plot(q)

disp 'yes.  In the zero-angle pose, axes q2,q3, and q5 are parallel'