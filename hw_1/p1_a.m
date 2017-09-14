% Chapter 2 exercise 2
clc;
clear all
close all


%% a) Write a function to plot the edges of a cube centered at the origin

% vectors defining the eight vertices of a unit cube centered about [0 0 0]
% vert = [X Y Z 1]
vert_1 = [0.5 -0.5 -0.5 1];
vert_2 = [0.5 -0.5 0.5 1];
vert_3 = [-0.5 -0.5 0.5 1];
vert_4 = [-0.5 -0.5 -0.5 1];
vert_5 = [-0.5 0.5 -0.5 1];
vert_6 = [-0.5 0.5 0.5 1];
vert_7 = [0.5 0.5 0.5 1];
vert_8 = [0.5 0.5 -0.5 1];

% put together a matrix that plots nicely
A1 = [vert_1;
     vert_2;
     vert_3;
     vert_4;
     vert_5;
     vert_6;
     vert_7;
     vert_8;
     vert_1;
     vert_4;
     vert_3;
     vert_6;
     vert_5;
     vert_8;
     vert_7;
     vert_2];

% plot the cube
figure(1)
plot3(A1(:,1),A1(:,2),A1(:,3),'k')
hold on
trplot
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
title('Figure for part a')
axis equal
axis([-3 3 -3 3 -3 3])
grid on

%% b) modify the function to accept a homogeneous transform before plotting

% define the transformation T
T = transl(2,2,1)*trotz(pi/4)*troty(-pi/4)*trotx(pi/6);

% create the new transformed vertices
vert_1_new = T*vert_1';
vert_2_new = T*vert_2';
vert_3_new = T*vert_3';
vert_4_new = T*vert_4';
vert_5_new = T*vert_5';
vert_6_new = T*vert_6';
vert_7_new = T*vert_7';
vert_8_new = T*vert_8';

% put together a matrix that plots nicely
A2 = [vert_1_new';
     vert_2_new';
     vert_3_new';
     vert_4_new';
     vert_5_new';
     vert_6_new';
     vert_7_new';
     vert_8_new';
     vert_1_new';
     vert_4_new';
     vert_3_new';
     vert_6_new';
     vert_5_new';
     vert_8_new';
     vert_7_new';
     vert_2_new'];
 
% plot the transformed cube and the original cube
figure(2)
plot3(A2(:,1),A2(:,2),A2(:,3),A1(:,1),A1(:,2),A1(:,3),'k')
hold on
trplot(T, 'frame', '2', 'color', 'r')
trplot
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
title('Figure for part b, c, d')
legend('transformed','original')
axis equal
axis([-3 3 -3 3 -3 3])
grid on

%% c) Animate rotation about the x-axis
% done
%% d) Animate rotation about all axes
% done

 
