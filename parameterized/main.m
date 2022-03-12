clc; clear all; close all;

addpath("../robots")
addpath("../math")
global m g Nx Nu
% global Pc Rc Fm
% global D d Ruu ru invR Qxx qx q0
% global A B C

%% set configuration matrices
robot.footname = {'al','ar','bl','br'};
robot = setRobotConfig(robot, 3); % 3rd step motion
m = 5;
g = [0,0,-9.8]';
Nx = 9; Nu = 9;

%% get robot system matrices
% let p(t) = Pcom + delP*(2s-1)

% A*Fc + S(s,ssdot)*delP + C = 0

[Pc, Rc, Fm] = setConfigMatices(robot);
[A, C] = setParameterizedSystem(robot.pcom,Pc,Rc,Fm);

%% set inequality constraints
% D*Fc - d >= 0
[D, d] = setFrictionCone(robot);
mu_ = setFricCoeff(robot);

%%
% given W
w=[];
for i=1:length(mu_)
    w = [w; 1./mu_(i);1./mu_(i);mu_(i)];
end
W = diag(w);
invW = diag(1./w);
invA = invW*A'*pinv(A*invW*A');

% given S
T = 0.3; timestep=0.01;
[s_, sdot_, sddot_] = hermiteS(T, timestep);

% find delP s.t. -D*invA*(S*delP+C)-d>0
% find delP s.t. DD*delP - dd >=0
dd = D*invA*C + d;


for i=1:length(s_)
    S = setHermiteMatrix(s_(i), sddot_(i), robot.pcom);
    DD = -D*invA*S;
    
    %% min 0.5*(Norm(C*delP-d)).^2 s.t. Ax<b : lsqlin(C,d,A,b)
    delP = lsqlin(eye(3),zeros(3,1),-DD,-dd)
    delP/norm(delP)
%     delP2 = quadprog(eye(3),zeros(3,1),-DD,-dd)
end




