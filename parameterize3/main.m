clc; clear all; close all;

addpath("../robots")
addpath("../math")
global m g Nx Nu
% global Pc Rc Fm
% global D d Ruu ru invR Qxx qx q0
% global A B C

%% set configuration matrices
robot.footname = {'al','ar','bl','br'};
m = 5;
g = [0,0,-9.8]';
% robot = setRobotConfig(robot, 4); % 3rd step motion
% pcom_goal = robot.pcom;

robot = setRobotConfig(robot, 3); % 3rd step motion
Nx = 9; Nu = 9;

pcom_goal = robot.pcom + 0.1*[-cos(1);0;sin(1)];
Ts = 0.35;
[delP, sddot] = findPhaseSwing(robot, pcom_goal, Ts)




%% check full support phase

p0 = robot.pcom;
p1 = pcom_goal-delP;
p2 = pcom_goal;
v0 = zeros(3,1);
v1 = 2/Ts*delP;
v2 = zeros(3,1);

T = 0.4;
p_full = [];
v_full = [];
a_full = [];
for tt = 0:0.01:T
p = hermite(tt, p0, p1, v0, v1, T);
v = hermite1stDeriv(tt, p0, p1, v0, v1, T);
a = hermite2ndDeriv(tt, p0, p1, v0, v1, T);

p_full = [p_full,p];
v_full = [v_full,v];
a_full = [a_full,a];
end

mid_idx = size(p_full,2);

for tt = 0.01:0.01:Ts
p = hermite(tt, p1, p2, v1, v2, Ts);
v = hermite1stDeriv(tt, p1, p2, v1, v2, Ts);
a = hermite2ndDeriv(tt, p1, p2, v1, v2, Ts);

p_full = [p_full,p];
v_full = [v_full,v];
a_full = [a_full,a];
end

close all;


figure;
subplot(3,1,1); plot(v_full(1,:));
subplot(3,1,2); plot(v_full(2,:));
subplot(3,1,3); plot(v_full(3,:));

figure;
subplot(3,1,1); plot(a_full(1,:));
subplot(3,1,2); plot(a_full(2,:));
subplot(3,1,3); plot(a_full(3,:));

figure;
plot3(p_full(1,:),p_full(2,:),p_full(3,:)); hold on;
plot3(p_full(1,1),p_full(2,1),p_full(3,1),'ro');
plot3(p_full(1,mid_idx),p_full(2,mid_idx),p_full(3,mid_idx),'rx');
grid on;
axis equal;
xlabel('x');
