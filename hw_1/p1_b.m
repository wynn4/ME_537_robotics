% Chapter 2 Exercise 4
clc;
clear all
close all

%% Create a 2D rotation matrix.

% angle to rotate by, theta
theta = pi/6;

% create the rotation matrix manually
disp 'rotation from frame 1 to frame 0:'
R_1_0 = [cos(theta) -sin(theta);
         sin(theta) cos(theta)]
     
 % create the same rotation matrix using the toolbox
 R = rot2(theta);
 
 % check if the rotations are equal
 if isequal(R_1_0,R)
     disp 'success!'
 else
     disp 'rotations not equivalent'
 end
     
 
 %% Visualize the rotation using trplot2
 trplot2(R_1_0)
 
 %% use the rotation to transform a vector
 vec_in_frame_1 = [1 1]';
 vec_in_frame_0 = R_1_0*vec_in_frame_1;
 
 %% Invert the rotation and and then multiply by the original rotation
 disp 'R_inv*R is:'
 R_inv = inv(R_1_0);
 result = R_inv*R_1_0
 disp 'result is identity'
 
 %% reverse the order and what is the result?
 disp 'R*R_inv is:'
 result = R_1_0*R_inv
 disp 'same result'
 
 %% what is the determinant of the rotation and its inverse?
 disp 'det(R) and det(R_inv):'
 det_R = det(R)
 det_R_inv = det(R_inv)
 