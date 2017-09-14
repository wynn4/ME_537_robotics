% Chapter 2 Exercise 16
clc;
clear all
close all

%% Is the inverse of a homogeneous transformation matrix equal to its transpose?

T = transl(2,2,1)*trotz(pi/4)*troty(-pi/4)*trotx(pi/6);

T_inv = inv(T)
T_trans = T'

if isequal(T_inv, T_trans)
    disp 'yes, T_inv == T_trans'
else
    disp 'no, T_inv != T_trans'
end