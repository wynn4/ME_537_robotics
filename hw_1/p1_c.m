% Chapter 2 Exercise 5
clc;
clear all
close all

%% Create a 3D rotation matrix

phi = pi/6;
theta = -pi/4;
psi = pi/2;

R_1_0 = rotz(psi)*roty(theta)*rotx(phi)

%% Create a 3D rotation matrix using trplot or tranimate

trplot(R_1_0)
% tranimate(R_1_0)

%% Transform a vector using the rotation matrix
vec_in_frame_1 = [1 1 1]'
vec_in_frame_0 = R_1_0*vec_in_frame_1

%% Invert the rotation and multiply by the original
R_inv = inv(R_1_0);
result = R_inv*R_1_0

%% Reverse the multiplication and what is the result?
result = R_1_0*R_inv

%% Determinant of rotation and its inverse
det_R = det(R_1_0)
det_R_inv = det(R_inv)