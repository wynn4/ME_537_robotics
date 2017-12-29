clc
clear
close all

% load puma robot
mdl_puma560

p560 = p560.nofriction();

% load camera
cam = CentralCamera('default');
% cam = CentralCamera('focal', 0.015, 'pixel', 10e-6, ...
% 'resolution', [1280 1024], 'centre', [640 512], 'name', 'mycamera');

P = mkcube(0.2, 'pose', transl([0.72, -0.15, -0.9]) );    % make a cube of points

% initial joint angles
q0 = [degtorad(0), degtorad(70), degtorad(-160), 0, degtorad(-70), 0];

% final joint angles
qfinal = [degtorad(0), degtorad(0), degtorad(-135), 0, degtorad(-45), 0];

figure(1), clf
p560.plot(q0);

% get end effector poses
T_init = p560.fkine(q0).T;
T_final = p560.fkine(qfinal).T;

% stick the camera at the end effector
cam.T = T_init;%*trotz(-pi/2);

cam.plot_camera;

% plot the points in the image plane
cam.plot(P)
pause;

% see how the points look in the final configuration
p560.plot(qfinal);
cam.T = T_final;%*trotz(-pi/2);
cam.plot_camera;
cam.plot(P)

% looks good! now let's do this

p = cam.project(P);
T_est = cam.estpose(P, p).T;
T_est = inv(T_est); % invert to put it in the right frame

% reset the robot and camera to q0
T = p560.fkine(q0).T;
plot_sphere(P, 0.03, 'r');  % display the cube
p560.plot(q0);
cam.T = T;

% lots of error to begin with
error = norm(tr2delta(T_init, T_final));

% implement PBVS in a loop
while error > 0.01
    pause(0.2);
    
    % get pixel projections
    p = cam.project(P);
    
    % get estimated transformation
    T_est = cam.estpose(P, p).T;
    T_est = inv(T_est); % invert to put it in the right frame
    
    % get transformation to step towards the goal based on the estimate
    T_step = trinterp(T_est, T_final, 0.1);
    
    % get the appropriate joint angles to step to using IK
    q = p560.ikine6s(T_step);
    
    % Update the robot and cameras positions
    T = p560.fkine(q).T;
    cam.T = T;
    
    % update the error
    error = norm(tr2delta(T, T_final))
    
    % update the plots
    p560.plot(q);
    cam.plot_camera;
    cam.plot(P) 
end

disp("done!")
