clc
clear
close all

% load baxter
[left, right] = mdl_baxter('real');

q = [0 0 0 0 0 0 0];

left.plot(q);
view(70, 25)