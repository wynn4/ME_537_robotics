% Chapter 2 Exercise 10
clc;
clear all
close all

%% Create a 2D or 3D homogeneous transformation matrix

T = transl(2,2,1)*trotz(pi/4)*troty(-pi/4)*trotx(pi/6);

%% Visualize the rigid-body displacement using tranimate

tranimate(T)

%% Use the transformation matrix to transform a vector
vec = [1 2 3 1]'
transformed_vec = T*vec

%% Invert the transformation and multiply by the original matrix
T_inv = inv(T);
result = T_inv*T

%% Switch the order of multiplication and now what is the result?
result = T*T_inv