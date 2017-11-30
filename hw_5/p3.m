% problem 3
clc
clear

load('puma560_torque_profile.mat');
load('sim_vars.mat')

% load the puma robot
mdl_puma560;

% QDD = R.accel(Q, QD, TORQUE) is a vector (Nx1) of joint accelerations that result 
%   from applying the actuator force/torque (1xN) to the manipulator robot R in
%   state Q (1xN) and QD (1xN), and N is the number of robot joints.
%  
%   If Q, QD, TORQUE are matrices (KxN) then QDD is a matrix (KxN) where each row 
%   is the acceleration corresponding to the equivalent rows of Q, QD, TORQUE.

% for i=1:length(q_sim)
%     
%     qdd1 = p560.accell(q_sim(i,1), qdot_sim(i,1), torque(find(time==t(i),1),:);
% end


% ran out of time...should have started sooner! :(

%% plots
figure(1), clf
subplot(3,1,1)
plot(t_sim,q_sim(:,1))
hold on
plot(t_sim,q_sim(:,2))
hold on
plot(t_sim,q_sim(:,3))
hold on
plot(t_sim,q_sim(:,4))
hold on
plot(t_sim,q_sim(:,5))
hold on
plot(t_sim,q_sim(:,6))
title('Puma560 joint angles vs Time')
xlabel('time (s)')
ylabel('joint angle (rad)')
legend("q1", "q2", "q3", "q4", "q5", "q6")

% figure(2), clf
subplot(3,1,2)
plot(t_sim,qdot_sim(:,1))
hold on
plot(t_sim,qdot_sim(:,2))
hold on
plot(t_sim,qdot_sim(:,3))
hold on
plot(t_sim,qdot_sim(:,4))
hold on
plot(t_sim,qdot_sim(:,5))
hold on
plot(t_sim,qdot_sim(:,6))
title('Puma560 joint velocity vs Time')
xlabel('time (s)')
ylabel('joint vel (rad/s)')
legend("q1", "q2", "q3", "q4", "q5", "q6")

% figure(3), clf
subplot(3,1,3)
plot(t_sim,qddot_sim(:,1))
hold on
plot(t_sim,qddot_sim(:,2))
hold on
plot(t_sim,qddot_sim(:,3))
hold on
plot(t_sim,qddot_sim(:,4))
hold on
plot(t_sim,qddot_sim(:,5))
hold on
plot(t_sim,qddot_sim(:,6))
title('Puma560 joint accel vs Time')
xlabel('time (s)')
ylabel('joint accel (rad/ss)')
legend("q1", "q2", "q3", "q4", "q5", "q6")
