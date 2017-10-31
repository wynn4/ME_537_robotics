function q_s = compute_robot_path(robot, robot3, robot2, q_init, goal, obst_location, obst_radius)

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
q_final = q;
% now we know where we need to end up, now lets figure out how to get
% there...

% check to see if I can rotate about q1 freely w/o hitting the obstacle
% using forward kinematics
q1_spin_check_angles = [3*pi/4, pi/2, pi/4, 0, -pi/4, -pi/2, -3*pi/4];
q1_spin_status = zeros(1,length(q1_spin_check_angles)*3);
count = 0;
for i = 1:length(q1_spin_check_angles)
    % get EE position
    Tee = robot.fkine([q1_spin_check_angles(i), q_init(2), q_init(3), q_init(4)]).T;
    pos_ee = Tee(1:3,4);
    % get joint 4 position
    Tj4 = robot3.fkine([q1_spin_check_angles(i), q_init(2), q_init(3)]).T;
    pos_j4 = Tj4(1:3,4);
    % get joint 3 position
    Tj3 = robot2.fkine([q1_spin_check_angles(i), q_init(2)]).T;
    pos_j3 = Tj3(1:3,4);
    
    three_check_positions = [pos_ee, pos_j4, pos_j3];
    
    % check to see if the distance between the point on the robot and the
    % obstacle is greater than obst_radius
    for j = 1:3
        count = count + 1;
        if norm(obst_location' - three_check_positions(:,j)) > obst_radius
            q1_spin_status(count) = 0;
        else
            q1_spin_status(count) = 1;
        end
    end
    
end

% check to see if I can rotate about q2 freely w/o hitting the obstacle
% using forward kinematics
q2_spin_check_angles = [3*pi/4, pi/2, pi/4, 0, -pi/4, -pi/2, -3*pi/4];
q2_spin_status = zeros(1,length(q2_spin_check_angles)*3);
count = 0;
for i = 1:length(q2_spin_check_angles)
    % get EE position
    Tee = robot.fkine([q_init(1), q2_spin_check_angles(i), q_init(3), q_init(4)]).T;
    pos_ee = Tee(1:3,4);
    % get joint 4 position
    Tj4 = robot3.fkine([q_init(1), q2_spin_check_angles(i), q_init(3)]).T;
    pos_j4 = Tj4(1:3,4);
    % get joint 3 position
    Tj3 = robot2.fkine([q_init(1), q2_spin_check_angles(i)]).T;
    pos_j3 = Tj3(1:3,4);
    
    three_check_positions = [pos_ee, pos_j4, pos_j3];
    
    % check to see if the distance between the point on the robot and the
    % obstacle is greater than obst_radius
    for j = 1:3
        count = count + 1;
        if norm(obst_location' - three_check_positions(:,j)) > obst_radius
            q2_spin_status(count) = 0;
        else
            q2_spin_status(count) = 1;
        end
    end
    
end

% decide if we're going to spin away from the obstacle using q1 or q2
if norm(q1_spin_status) < 1 && norm(q2_spin_status) >= 1
    avoidance_type = 1;
elseif norm(q2_spin_status) < 1 && norm(q1_spin_status) >= 1
    avoidance_type = 2;
else
    avoidance_type = 0; % just crash into it
end

% figure out the xy and yz headings to the obstacle --save for later
obst_heading_xy = atan2(obst_location(2),obst_location(1)) - pi/2;
obst_heading_yz = atan2(obst_location(3),obst_location(2)) - pi/2;

% split up where we are to where we need to go into little chunks

% do some checks first to see if the initial and final joint angles are
% different
flag = zeros(4,1);
flag_rev = zeros(4,1);
for i = 1:4
    % if so, set a flag
    if norm(q_final(i) - q_init(i)) < 0.01
        flag(i) = 1;
    end
    
    % see if we should split up the vector in reverse order
    if q_final(i) < q_init(i)
        flag_rev(i) = 1;
    end
end

% if initial and final joint angles were the same...
if flag(1) == 1
    q1 = zeros(1,10);   % just a bunch of zeros
elseif flag_rev(1) == 1
    q1 = q(1):0.02:q_init(1);   % chop up in tiny pieces from init to final
    q1 = fliplr(q1);
else
    q1 = q_init(1):0.02:q(1);
end

% if initial and final joint angles were the same...
if flag(2) == 1
    q2 = zeros(1,10);   % just a bunch of zeros
elseif flag_rev(2) == 1
    q2 = q(2):0.02:q_init(2);   % chop up in tiny pieces from init to final
    q2 = fliplr(q2);
else
    q2 = q_init(2):0.02:q(2);
end

% if initial and final joint angles were the same...
if flag(3) == 1
    q3 = zeros(1,10);   % just a bunch of zeros
elseif flag_rev(3) == 1
    q3 = q(3):0.02:q_init(3);   % chop up in tiny pieces from init to final
    q3 = fliplr(q3);
else
    q3 = q_init(3):0.02:q(3);
end

% if initial and final joint angles were the same...
if flag(4) == 1
    q4 = zeros(1,10);   % just a bunch of zeros
elseif flag_rev(4) == 1
    q4 = q(4):0.02:q_init(4);   % chop up in tiny pieces from init to final
    q4 = fliplr(q4);
else
    q4 = q_init(4):0.02:q(4);
end



% make q_s huge!! (memory allocation)
q_s = zeros(100000,4);

% avoidance_type = 1;
% if the path is clear to spin about q1...
if avoidance_type == 1
    q_avoid = q_init(1):0.02:obst_heading_xy+(pi/2);   % going to rotate 90 degrees past where the obsticle is
    % initialize another counter
    count = 0;
    
    % move joints sequentially
    % first do the avoidance manuver on joint 1
    for i = 1:length(q_avoid)
        count = count + 1;
        q_s(count,:) = [q_avoid(i) q_init(2) q_init(3) q_init(4)];
    end
    q_avoid = fliplr(q_avoid);  % set q_avoid to undo the avoidance manuver
    q1_last = q_s(count,1); % done with the avoidance on joint 1
    
    % joint 2
    for i = 1:length(q2)
        count = count + 1;
        q_s(count,:) = [q1_last q2(i) q_init(3) q_init(4)];
    end
    q2_final = q_s(count,2);    % done moving joint 2
    
    % joint 3
    for i = 1:length(q3)
        count = count + 1;
        q_s(count,:) = [q1_last q2_final q3(i) q_init(4)];
    end
    q3_final = q_s(count,3);    % done moving joint 3
    
    % joint 4
    for i = 1:length(q4)
        count = count + 1;
        q_s(count,:) = [q1_last q2_final q3_final q4(i)];
    end
    q4_final = q_s(count,4);
    
    % undo avoidance on joint 1
    if q1_last < q_final(1)
        q_undo = q1_last:0.02:q_final(1);
    elseif q_final(1) < q1_last
        q_undo = q_final(1):0.02:q1_last;
        q_undo = fliplr(q_undo);
    else
        q_undo = q_avoid;
    end
    for i = 1:length(q_undo)
        count = count + 1;
        q_s(count,:) = [q_undo(i) q2_final q3_final q4_final];
    end
    
    % done moving all the joints. Now trim off the excess rows of q_s
    q_s = q_s(1:count,:);
    return
end

% if the path is clear to spin about q2...
if avoidance_type == 2
    q_avoid = q_init(2):0.02:obst_heading_yz+(pi/2);   % going to rotate 90 degrees
    % initialize another counter
    count = 0;
    % joint 1
    % move joints sequentially
    % first, do the avoidance manuver on joint 2
    for i = 1:length(q_avoid)
        count = count + 1;
        q_s(count,:) = [q_init(1) q_avoid(i) q_init(3) q_init(4)];
    end
    q_avoid = fliplr(q_avoid);  % set q_avoid to undo the avoidance manuver
    q2_last = q_s(count,2);
    
    % now tuck in the End-Effector --just in case
    q4_tuck_ang = pi;   % joint angle to keep EE tucked in
    if q_init(4) > q4_tuck_ang
        q4_tuck = q4_tuck_ang:0.02:q_init(4);
        q4_tuck = fliplr(q4_tuck);
    else
        q4_tuck = q_init(4):0.02:q4_tuck_ang;
    end
    
    for i = 1:length(q4_tuck)
        count = count + 1;
        q_s(count,:) = [q_init(1) q2_last q_init(3) q4_tuck(i)];
    end
    q4_tucked = q_s(count,4);   % done tucking in the end effector
    
    % move joint 1
    for i = 1:length(q1)
        count = count + 1;
        q_s(count,:) = [q1(i) q2_last q_init(3) q4_tucked];
    end
    q1_final = q_s(count,1);    % done moving joint 1
   
    % joint 3
    for i = 1:length(q3)
        count = count + 1;
        q_s(count,:) = [q1_final q2_last q3(i) q4_tucked];
    end
    q3_final = q_s(count,3);    % done moving joint 3
    
    % joint 4
    % undo the tuck on EE joint 4
    if q4_tucked < q_final(4)
        q_tuck_undo = q4_tucked:0.02:q_final(4);
    elseif q_final(4) < q4_tucked
        q_tuck_undo = q_final(4):0.02:q4_tucked;
        q_tuck_undo = fliplr(q_tuck_undo);
    end
    
    for i = 1:length(q_tuck_undo)
        count = count + 1;
        q_s(count,:) = [q1_final q2_last q3_final q_tuck_undo(i)];
    end
    q4_final = q_s(count,4);
    
    % undo avoidance manuver on joint 2
    if q2_last < q_final(2)
        q_undo = q2_last:0.02:q_final(2);
    elseif q_final(2) < q2_last
        q_undo = q_final(2):0.02:q2_last;
        q_undo = fliplr(q_undo);
    else
        q_undo = q_avoid;
    end
    % fill in q_s with the appropriate undo manuver
    for i = 1:length(q_undo)
        count = count + 1;
        q_s(count,:) = [q1_final q_undo(i) q3_final q4_final];
    end

    q_s = q_s(1:count,:);
    return
end

% if the path isn't clear in either direction, or its clear in both, just
% blindly go to the point by moving joints one by one
if avoidance_type == 0
    count = 0;
    % move joint 1
    for i = 1:length(q1)
        count = count + 1;
        q_s(count,:) = [q1(i) q_init(2) q_init(3) q_init(4)];
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
    
    % done moving all the joints. Now trim off the excess rows of q_s
    q_s = q_s(1:count,:);
    return
end

end