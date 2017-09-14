% Chapter 2 Exercise 20
clc;
clear all
close all

%% A camera's z-axis is pointing in the [0, 1, 0] dir and it's y-axis in the [0, 0, -1] dir

% given
z_cam = [0 1 0];
y_cam = [0 0 -1];

% which also means:
x_cam = [1 0 0];

% define the world axes
x_world = [1 0 0];
y_world = [0 1 0];
z_world = [0 0 1];

%% What is the camera's attitude in the world frame expressed as a rotation matrix

R_cam_world = [dot(x_cam, x_world) dot(y_cam, x_world) dot(z_cam, x_world);
               dot(x_cam, y_world) dot(y_cam, y_world) dot(z_cam, y_world);
               dot(x_cam, z_world) dot(y_cam, z_world) dot(z_cam, z_world)]
           
%% What is the camera's attitude expressed as a unit quaternion?

q = UnitQuaternion(R_cam_world)