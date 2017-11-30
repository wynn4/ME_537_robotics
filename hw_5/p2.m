% problem 2 baxter

clc
clear
close all

load('desired_accel.mat')

[left, right] = mdl_baxter('real');
clc

Ts = 0.01; % time step

% we have q's, need qdot and qddot's
q1dot = diff(q(:,1))/Ts;
q2dot = diff(q(:,2))/Ts;
q3dot = diff(q(:,3))/Ts;
q4dot = diff(q(:,4))/Ts;
q5dot = diff(q(:,5))/Ts;
q6dot = diff(q(:,6))/Ts;
q7dot = diff(q(:,7))/Ts;

qdot = [q1dot q2dot q3dot q4dot q5dot q6dot q7dot];

q1ddot = diff(qdot(:,1))/Ts;
q2ddot = diff(qdot(:,2))/Ts;
q3ddot = diff(qdot(:,3))/Ts;
q4ddot = diff(qdot(:,4))/Ts;
q5ddot = diff(qdot(:,5))/Ts;
q6ddot = diff(qdot(:,6))/Ts;
q7ddot = diff(qdot(:,7))/Ts;

qddot = [q1ddot q2ddot q3ddot q4ddot q5ddot q6ddot q7ddot];

% chop off so that dimensions line up
q = q(3:length(q),:);
qdot = qdot(2:length(qdot),:);
t = t(3:length(t),:);

% now go through and calculate the joint torques
tau_joints = zeros(length(q),7);

count = 0;
for i=1:length(q)
    
    % get M(q)
    M = right.inertia(q(i,:));
    
    % get C(q,qdot)
    C = right.coriolis(q(i,:), qdot(i,:));
    
    % G(q)
    G = right.gravload(q(i,:));
    
    % compute joint torques
    T = M * qddot(i,:)' + C * qdot(i,:)' + G * q(i,:)';
    
    % add to our tau matrix
    tau_joints(i,:) = T';
    
    % see progress
    count = count + 1
end

%% plots
figure(1), clf
plot(t,tau_joints(:,1))
hold on
plot(t,tau_joints(:,2))
hold on
plot(t,tau_joints(:,3))
hold on
plot(t,tau_joints(:,4))
hold on
plot(t,tau_joints(:,5))
hold on
plot(t,tau_joints(:,6))
hold on
plot(t,tau_joints(:,7))
legend("q1", "q2", "q3", "q4", "q5", "q6", "q7")
title("Baxter Joint Torques vs Time")
xlabel("time (s)")
ylabel("Joint Torque (N/m)")