clc
clear
close all

% load baxter
mdl_baxter;

% the four q angle sets we used
q1 = [-1.60147594 -1.30196619 -0.95183508  0.23086411 -3.04571885 -0.4153253 3.03268002];
q2 = [-0.21015537 -1.09871374 -1.04847587  1.19995647 -3.04686934 -0.64197096 3.03498099];
q3 = [ 0.3831117  -0.59518455 -0.79460205  1.09794675 -2.02562163 -0.76430593 3.03574798];
q4 = [ 1.05998072 -0.25272333 -0.64043698  0.77887875  1.03045159  0.39001462 3.03536448];
q = [q1; q2; q3; q4];

% the four resultant robot poses from the python code
Tcode1 = [-0.64760949 -0.1636179  -0.74419831 -0.44446011;
          0.29937056 -0.95277072 -0.05104132 -0.41616949;
          -0.70069909 -0.25584591  0.66600575 1.2246117;
          0 0 0 1];

Tcode2 = [-0.12936671  0.86375504 -0.48702309 -0.0931718;
          0.60120831 -0.32225608 -0.73123155 -1.02720427;
          -0.78855109 -0.38739935 -0.4776075 0.64177675;
          0 0 0 1];
      
Tcode3 = [-0.16063797  0.84462847  0.51068404 0.67702823;
          0.85660293  0.37634011 -0.35298659 -0.87259534;
          -0.49033341  0.3807504  -0.78396574 0.28959711;
          0 0 0 1];
  
Tcode4 = [0.62085735 -0.24559001  0.74446068 1.00255158;
          -0.35264531 -0.93564372 -0.014564 -0.25071717;
          0.70012673 -0.25348841 -0.66750744 0.13183004;
          0 0 0 1];

% figure(1), clf
% 
% right.plot(q1)
% right.plot(q2)
% right.plot(q3)
% right.plot(q4)

% estimated robot poses using fkine()
T1 = right.fkine(q1)
T2 = right.fkine(q2);
T3 = right.fkine(q3);
T4 = right.fkine(q4);


Tdh1 = eye(4);
Tdh2 = eye(4);
Tdh3 = eye(4);
Tdh4 = eye(4);

d = [0.2704 0 0.36435 0 0.3743 0 0.2295];
a = [0.069 0 0.069 0 0.01 0 0];
alpha = [-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 0];
offset = [0 pi/2 0 0 0 0 0];

for i = 1:7
    Tdh1 = Tdh1 * [rotz(q1(i) + offset(i)), zeros(3,1); zeros(1,3), 1]*[eye(3), [0, 0, d(i)]'; zeros(1,3), 1]*[eye(3), [0, 0, a(i)]'; zeros(1,3), 1]*[rotx(alpha(i)), zeros(3,1); zeros(1,3), 1];
end

for i = 1:7
    Tdh2 = Tdh2 * [rotz(q2(i) + offset(i)), zeros(3,1); zeros(1,3), 1]*[eye(3), [0, 0, d(i)]'; zeros(1,3), 1]*[eye(3), [0, 0, a(i)]'; zeros(1,3), 1]*[rotx(alpha(i)), zeros(3,1); zeros(1,3), 1];
end

for i = 1:7
    Tdh3 = Tdh3 * [rotz(q3(i) + offset(i)), zeros(3,1); zeros(1,3), 1]*[eye(3), [0, 0, d(i)]'; zeros(1,3), 1]*[eye(3), [0, 0, a(i)]'; zeros(1,3), 1]*[rotx(alpha(i)), zeros(3,1); zeros(1,3), 1];
end

for i = 1:7
    Tdh4 = Tdh4 * [rotz(q4(i) + offset(i)), zeros(3,1); zeros(1,3), 1]*[eye(3), [0, 0, d(i)]'; zeros(1,3), 1]*[eye(3), [0, 0, a(i)]'; zeros(1,3), 1]*[rotx(alpha(i)), zeros(3,1); zeros(1,3), 1];
end

% should have used T = trotz(q) and T = trotx(alpha)
% also should have used T = transl([x y z])
% next time...


% compute some differences
diff1 = Tcode1 - Tdh1
diff2 = Tcode2 - Tdh2
diff3 = Tcode3 - Tdh3
diff4 = Tcode4 - Tdh4