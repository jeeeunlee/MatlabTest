clc; clear all; close all;

addpath("../robots")
addpath("../math")
global m g Nx Nu
% global Pc Rc Fm
% global D d Ruu ru invR Qxx qx q0
% global A B C

%% set configuration matrices
robot.footname = {'al','ar','bl','br'};
robot = setRobotConfig(robot, 4); % 3rd step motion
m = 5;
g = [0,0,-9.8]';

Nx = 9; Nu = 12;
[DD, dd] = findPhase3(robot);


robot = setRobotConfig(robot, 3); % 3rd step motion
Nx = 9; Nu = 9;
[DD, dd] = findPhase2(robot);





