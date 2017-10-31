function q_s = compute_robot_path(robot, q_init, goal, obst_location, obst_radius)

% define where we want the robot to go
% desired location
d = [goal(1) goal(2) goal(3)]';

% desired orientation (arbitrary...choose identity?)
orient = eye(3);

% construct desired final pose
T_des = [orient, d; 0 0 0 1];

% use Jacobian Transpose method to see if an IK solution exists
counter = 0;
K_trans = 0.5*eye(3);   % tuning param
T_cur = robot.fkine(q_init).T;
q = q_init';
while (norm(T_des(1:3,4)- T_cur(1:3,4)) > 0.05) && (counter < 1000)  % within 5 centimeter and we'll call it good
    % update pose and jacobian for current q's
    T_cur = robot.fkine(q).T;
    J = robot.jacob0(q);
    
    %calculate error in pose and transform back to base frame.
    delta = tr2delta(T_cur, T_des);
    R2base = T_cur(1:3,1:3);
    delta_base = [R2base, zeros(3,3); zeros(3,3), R2base]*delta;
    
    %perform the Jacobian-Transpose method
    q = q + J(1:3,:)'*K_trans*delta_base(1:3);
    
    disp('computing inverse kinematics...')
    counter = counter + 1
%     q_path(counter,:) = q';
end

if counter == 1000
    s = strcat("failed to find IK solution after ",int2str(counter), " iterations");
    disp(s);
    q_s = q_init;
    return
end

disp("IK solution found!")

% now we know where we need to end up, now lets make paths to get there
q_final = q;

where_i_am = robot.fkine(q_init).T;
where_i_am = where_i_am(1:3,4);

% first, figure out which case we're in
avoid_flag_1 = 0;
% if the obstacle is above where you start and below where you need to end
% up:
if obst_location(3) > where_i_am(3) && obst_location(3) < goal(3)
    avoid_flag_1 = 1;
end
% split up where we are to where we need to go into little chunks

% do some checks first to see if the initial and final joint angles are
% different
flag = zeros(4,1);
flag_rev = zeros(4,1);
for i = 1:4
    if norm(q_final(i) - q_init(i)) < 0.01
        flag(i) = 1;
    end
    
    % see if we should split up the vector in reverse order
    if q_final(i) < q_init(i)
        flag_rev(i) = 1;
    end
end

% if we need to follow an avoidance path:
if avoid_flag_1 == 1
    q_avoid_1 = q_init(1):0.1:q_init(1)+pi/2;
else
    q_avoid_1 = zeros(1,1);
end

% if initial and final joint angles were the same...
if flag(1) == 1
    q1 = zeros(1,10);   % just a bunch of zeros
elseif flag_rev(1) == 1
    q1 = q(1):0.01:q_init(1);   % chop up in tiny pieces from init to final
    q1 = fliplr(q1);
else
    q1 = q_init(1):0.01:q(1);
end

% if initial and final joint angles were the same...
if flag(2) == 1
    q2 = zeros(1,10);   % just a bunch of zeros
elseif flag_rev(2) == 1
    q2 = q(2):0.01:q_init(2);   % chop up in tiny pieces from init to final
    q2 = fliplr(q2);
else
    q2 = q_init(2):0.01:q(2);
end

% if initial and final joint angles were the same...
if flag(3) == 1
    q3 = zeros(1,10);   % just a bunch of zeros
elseif flag_rev(3) == 1
    q3 = q(3):0.01:q_init(3);   % chop up in tiny pieces from init to final
    q3 = fliplr(q3);
else
    q3 = q_init(3):0.01:q(3);
end

% if initial and final joint angles were the same...
if flag(4) == 1
    q4 = zeros(1,10);   % just a bunch of zeros
elseif flag_rev(4) == 1
    q4 = q(4):0.01:q_init(4);   % chop up in tiny pieces from init to final
    q4 = fliplr(q4);
else
    q4 = q_init(4):0.01:q(4);
end



% make q_s huge!! (memory allocation)
q_s = zeros(100000,4);

% initialize another counter
count = 0;
% joint 1
% move joints sequentially
for i = 1:length(q_avoid_1)
    count = count + 1;
    q_s(count,:) = [q_avoid_1(i) q_init(2) q_init(3) q_init(4)];
end
q_avoid_1 = fliplr(q_avoid_1);
q1_last = q_s(count,1);

for i = 1:length(q1)
    count = count + 1;
    q_s(count,:) = [q1(i) + q1_last q_init(2) q_init(3) q_init(4)];
end
q1_final = q_s(count,1);    % done moving joint 1

% joint 2
for i = 1:length(q2)
    count = count + 1;
    q_s(count,:) = [q1_final q2(i) q_init(3) q_init(4)];
end
q2_final = q_s(count,2);    % done moving joint 2

% joint 3
for i = 1:length(q3)
    count = count + 1;
    q_s(count,:) = [q1_final q2_final q3(i) q_init(4)];
end
q3_final = q_s(count,3);    % done moving joint 3

% joint 4
for i = 1:length(q4)
    count = count + 1;
    q_s(count,:) = [q1_final q2_final q3_final q4(i)];
end
q4_final = q_s(count,4);

% undo avoidance on joint 1
for i = 1:length(q_avoid_1)
    count = count + 1;
    q_s(count,:) = [q_avoid_1(i) q2_final q3_final q4_final];
end

% done moving all the joints. Now trim off the excess rows of q_s
q_s = q_s(1:count,:);


end