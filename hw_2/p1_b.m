% Jesse Wynn HW2 ME 537 Robotics
clc
clear all
close all

% Problem 12 from Chapter 2

% rpy (phi, theta, psi)
syms phi theta psi

% create a rotation from they roll pitch and yaw angles
R = rpy2r(phi, theta, psi)

% use the rotation to transform a unit vector in the z-dir
vec = [0 0 1]';

T = R*vec

% you can now figure out the solution using a system of equations