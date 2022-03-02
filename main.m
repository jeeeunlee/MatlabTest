clear all;
global D d R invR Qxx qx q0 Q Pc Rc m g Nx Nu
global A B C

%% set configuration matrices
robot.footname = {'ar','al','br','bl'};
robot =  setRobotConfig(robot, 3); % 3rd step motion

%% get robot system matrices
[Pc, Rc] = setConfigMatices(robot);
[A,C] =  setSystemMatrices(robot);
m = 5;
g = [0,0,-9.8]';


%% set inequality constraints
[D, d] = setFrictionCone(robot);

%% constrained LQR problem
% x = [L, Pg, dPg] in 9x1
% min intgral(l(x,u,t)) 
% s.t. x' = Ax + B(x)u + c
%      Du - d >= 0
% l(x,u,t) = (x-xg)'Q(x-xg) + u'Ru
% J(x,t) = [x; 1]'S(t)[x; 1] : const-to-go function

Nx = 9; Nu = 9;
R = eye(Nu); invR = R\eye(Nu);
Qxx =  eye(Nx); qx = -Qxx*xgoal; q0 = xgoal'*xgoal;
Q = [Qxx qx; qx' q0];


%% backward

x0 = [zeros(3,1); robot.pcom; zeros(3,1)];

B = computeB(x);
[u, Kxx, kx] = computeU(Sxx, sx, x);
[Sxxdot, sxdot, s0dot] = getSdot(Sxx, sx, Kxx, kx);