% HW_1 problem 5
clc;
clear all
close all

%% Compute the rotation given by the supplied product

% define some symbolic variables

syms theta phi psi 

R = rotx(theta)*roty(phi)*rotz(psi)*roty(-phi)*rotx(-theta)