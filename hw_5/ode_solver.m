function q_states = ode_solver(t, q_out, robot, torque, time)

disp(q_out)
% disp(length(q_out))
% disp(torque)
mid = length(q_out)/2;
ful = length(q_out);
tau = torque(find(time==t,1),:);
disp(tau)

M = robot.inertia(q_out(1:mid,1)');
C = robot.coriolis(q_out(1:mid,1)', q_out(mid+1:ful,1)');
G = robot.gravload(q_out(1:mid,1)');

q_dd = robot.accel(q_out(1:mid,1)', q_out(mid+1:ful,1)', tau);

q_all = [q_out(mid+1:ful,1); q_dd];