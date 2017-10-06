% Jesse Wynn
% Robotics HW 3

clc
clear

% Problem 7-4 from the book

% experiment with ikine and ikine6s with the p560 robot

mdl_puma560

bot = p560;

% different joint angles

q1 = [0, 0.7854, 3.141, 0, 0.7854, 0];
q2 = [0, 0.8, 3, 0, 0.8, 0];
q3 = [0, 0.75, 3.2, 0, 0.75, 0];

q = [q1; q2; q3];

for i = 1:3
    joint_angles = q(i,:);
    
    % get the transformation using forward kinematics
    T = bot.fkine(joint_angles);
    
    % now compare ikine6s and ikine (analytical and numerical)
    tstart_1 = tic;
    
    % solve using ikine6s
    ik6s_joint_angles = bot.ikine6s(T);
    
    telapsed_1 = toc(tstart_1);
    
    % now do the same with ikine
    tstart_2 = tic;
    
    % solve using ikine
    ik_joint_angles = bot.ikine(T);
    
    telapsed_2 = toc(tstart_2);
    
    if telapsed_2 > telapsed_1
        disp 'ikine faster than ikine6s by:'
        diff = (telapsed_2 - telapsed_1);
        disp(diff)
        disp 'seconds'
    else
        disp 'this will never happen'
    end
    
    % see how different the transformations are
    T_ik = bot.fkine(ik_joint_angles);
    
    T_diff = T_ik - T
end









