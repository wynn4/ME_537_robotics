clc
clear all
close all

% hw_7 problem 3

% load camera
cam = CentralCamera('default');

% load the data
load('hw7_prob3.mat');

% estimate the transformation between the world points and the camera
T_est_1 = cam.estpose(P1, p1)
T_est_2 = cam.estpose(P2, p2)
T_est_3 = cam.estpose(P3, p3)


% Tcam = transl(0,0,-0.5);
% 
% cam.plot(P1, 'pose', Tcam)