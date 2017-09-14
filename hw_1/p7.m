% HW_1 problem 7
clc;
clear all
close all

R_original_frame = [1 0 0;
                    0 1 0;
                    0 0 1];

R_new = rotz(pi/4)*roty(0)*rotx(pi/2)

figure(1)
trplot(R_original_frame, 'color', 'k')
hold on
trplot(R_new, 'color', 'r')

                