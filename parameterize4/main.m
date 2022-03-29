clc; clear all; close all;

addpath("../robots")
addpath("../math")
global m g Nx Nu
% global Pc Rc Fm
% global D d Ruu ru invR Qxx qx q0
% global A B C

%% set configuration matrices
robot.footname = {'al','ar','bl','br'};
m = 5.7835741;
g = [0,0,-9.81]';
step_index = 3;
robot_goal = setRobotConfig(robot, step_index+1); % 4th step motion = 3rd step motion done
Nx = 9; Nu = 12;
pcom_goal = robot_goal.pcom;
Tt = 0.05;
[DD1, DD2, ddf] = findPhaseSwingEnd(robot_goal, pcom_goal, Tt);

robot = setRobotConfig(robot, step_index); % 3rd step motion
Nx = 9; Nu = 9;

pcom_goal = robot.pcom + 0.025*[-cos(1);0;sin(1)];
Ts = 0.25;
% [DDas, DDbs, dds] = findPhaseSwing(robot, pcom_goal, Ts, Tt, 0); %t=0~Ts


% ratio = -0.3 %[ -10, -5, -2 ]
% ratio = - ( Tt*Tt + 2*(Ts+Tt)*Tt) /(Ts+Tt)/(Ts+Tt)
a=0.5*(Ts+Tt)*(Ts+Tt);
c=-0.5*Tt*Tt;
b=(Ts+Tt)*Tt+Tt*Tt;
ratio = (-b - sqrt(b*b-4*a*c)) / 2/a

% ratio = 0

[DDas, DDbs, dds] = findPhaseSwingGivenRatio(robot, pcom_goal, Ts+Tt, Tt, ratio);

DD = [DD1; DD2];
dd = [ddf; ddf];
DD = [DD; DDas*ratio+DDbs];
dd = [dd; dds];

beta_d = quadprog(eye(3),zeros(3,1),-DD,-dd)
beta = - norm(beta_d)
d = beta_d/beta;

alpha = beta*ratio

sv3 = -beta*Tt;
sv2 = -beta*Tt -alpha*(Tt+Ts);
    
        

%% check full support phase

p0 = robot.pcom;
p3 = pcom_goal;
p2 = p3 + ( 0.5*beta*Tt*Tt )*d; 
% p1 = p2 - ( + 0.5*alpha*Ts*Ts + Ts*(-beta*Tt-alpha*Ts) )*d;
p1 = p2 + (beta*(Ts+Tt)*Tt + 0.5*alpha*(Ts+Tt)*(Ts+Tt))*d;
 
v0 = zeros(3,1);
v1 = sv2*d;
v2 = sv3*d;
v3 = zeros(3,1);

Tf = 0.2;

plottraj;
