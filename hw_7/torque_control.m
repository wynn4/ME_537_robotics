function tau = torque_control(theta_des, theta, theta_dot)

kp = 1;
kd = 0.1;

tau = kp*(theta_des - theta) - kd*(theta_dot);
end