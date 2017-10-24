% Jesse Wynn
% Homework 4
% Problem 3

clc
close all
clear

% load in the 10 random poses
load('random_poses.mat')

% load in the corresponding joint angles
load('q_angles.mat')

% load the robot
robot = robot_6dof();

% two different starting sets of joint angles
q1 = [0.0 0.0 0.0 0.0 0.0 0.0];
q2 = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2];

q_initials = [q1; q2];

rows = size(q_initials);
rows = rows(1);

cur_pos_vec = zeros(3,1);

% IK solution finding methods

% error threshold
error_thresh = 0.01;  % 10mm

% time step
dt = 0.01;

% method 2: Jacobian Transpose

error = 100;
count = 0;

% damped inverse scalar
kd = 0.1;

% error gain
K = 10 .* eye(6);
K = K .* dt;    % multiply by dt

% loop through the 10 random desired poses
for i = 1:10
    
    T_des = robot.fkine(q_angles(i,:));
    
    % for the two initial joint configurations
    for j = 1:rows
        
        % initialize variable q
        q = q_initials(j,:);
        
        % loop to find IK solution to T_des
        while error > error_thresh
            
            % compute Jacobian
            J = robot.jacob0(q);
            %     J = J(1:3,:);    % just get the first 3 rows
            
            % get current pose
            T_cur = robot.fkine(q);
            
            % get the error between the current T and desired T
            e = tr2deltabase(T_cur, T_des);
            
            % compute new q and continue
            q = q + J'/(J*J'+kd^2*eye(6))*(K*e);    % '/' is same as inv()
            
            q = diag(q)';   % just grab the diagonal elements (hopefully this works)
            
            error = norm(e);    % this seems like an OK way to quantify error
            
            count = count + 1;
            disp(count)
        end
        
        % reset count and error
        count = 0;
        error = 100;
    end
end

