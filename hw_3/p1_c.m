% Jesse Wynn
% Robotics HW 3

clc
% clear

% Problem 7-6 from the book

% investigate manufacturing error in the puma560

%% link 2 0.005 mm too long

joint_deflections = -0.7:0.014:0.7;

positions_w_error = zeros(length(joint_deflections),3);

error_length_norm = zeros(length(joint_deflections),1);

positions_w_joint_angle_error = zeros(length(joint_deflections),3);

for i = 1:length(joint_deflections)
    
    deflection = joint_deflections(i);
    
    q = [0, 0, 0, 0, 0, 0];
    
    q(1) = q(1) + deflection;
    q(2) = q(2) + deflection;
    q(3) = q(3) + deflection;
    q(4) = q(4) + deflection;
    q(5) = q(5) + deflection;
    q(6) = q(6) + deflection;
    
    T = p560.fkine(q);
    
    % lets just look at the components of ee position
    x = T.t(1);
    y = T.t(2);
    z = T.t(3);
    
    p = [x, y, z];
    
    % add the position to our positions matrix
    positions_w_error(i,:) = p;
    
end

%% error analysis

% error due to manufacturing error in length
error_length = positions_true - positions_w_error;

% error due to joint angle 2 being off by 0.1 degrees
error_angle = positions_true - positions_w_joint_angle_error;

% go through the error_length matrix and get the norm of the error for each
% joint configuration
for i = 1:length(joint_deflections)
    % length error
    er_norm_len = norm(error_length(i,:));
    
    error_length_norm(i) = er_norm_len;
    
    % angle error
    er_norm_ang = norm(error_angle(i,:));
    
    error_angle_norm(i) = er_norm_ang;
end

% what is the mean error?
mean_error_due_to_length = mean(error_length_norm)

% what is the max error?
max_error_due_to_length = max(error_length_norm)

% what is the mean error?
mean_error_due_to_angle = mean(error_angle_norm)

% what is the max error?
max_error_due_to_angle = max(error_angle_norm)
%%

