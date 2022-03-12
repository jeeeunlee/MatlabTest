clc; clear all; close all;
addpath('../math');

[s_, sdot_, sddot_] = hermiteS(1, 0.001);

figure; subplot(3,1,1); plot(s_);
subplot(3,1,2); plot(sdot_);
subplot(3,1,3); plot(sddot_);