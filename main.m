clear all; close all;
global D d R invR Qxx qx q0 Q Pc Rc m g Nx Nu Fm
global A B C

%% set configuration matrices
robot.footname = {'al','ar','bl','br'};
robot =  setRobotConfig(robot, 3); % 3rd step motion
m = 5;
g = [0,0,-9.8]';

contactfoot={''}; j=1;
for i=1:length(robot.footname)
    fn = robot.footname{i};
    if(~strcmp(robot.swing,fn))
        contactfoot{j} = fn;
        j = j+ 1;
    end
end


%% get robot system matrices
[Pc, Rc, Fm] = setConfigMatices(robot);
[A,C] =  setSystemMatrices();


%% set inequality constraints
[D, d] = setFrictionCone(robot);

%% constrained LQR problem
% x = [L, Pg, dPg] in 9x1
% min intgral(l(x,u,t)) 
% s.t. x' = Ax + B(x)u + c
%      Du - d >= 0
% l(x,u,t) = (x-xg)'Q(x-xg) + u'Ru
% J(x,t) = [x; 1]'S(t)[x; 1] : const-to-go function

xgoal = [zeros(3,1); robot.pcom; zeros(3,1)];

Nx = 9; Nu = 9;
% R = eye(Nu); 
R = setWeightMatrices(robot);
invR = R\eye(Nu);
% Qxx =  100000*eye(Nx); 
wL = 10000;
wP = 1;
wdP = 100;
Qxx = diag([wL,wL,wL, wP,wP,wP, wdP,wdP,wdP]);
qx = -Qxx*xgoal; q0 = xgoal'*Qxx*xgoal;
% Q = [Qxx qx; qx' q0];


%% backward propagation to get Sxx, sx, Kxx, kx
tstep = 0.001;
Nstep = 200;

x0 = [zeros(3,1); robot.pcom; zeros(3,1)];
B = computeB(x0);


Sxx_seq = [];
sx_seq = [];
% Sxx = zeros(Nx);
% sx = zeros(Nx,1);
Sxx = pinv(B*invR*B')*A;
sx = pinv(B*invR*B')*C;

for tt = 1:Nstep
    [u, Kxx, kx] = computeU(Sxx, sx, x0);
    [Sxxdot, sxdot, s0dot] = getSdot(Sxx, sx, Kxx, kx);
    
    xdot = A*xgoal + B*u + C;
    ddpg = 1/m*(Rc*(u+Fm))+g;
    
    Sxx = Sxx - Sxxdot*tstep;
    sx = sx - sxdot*tstep;
    % stack backward
    Sxx_seq = [reshape(Sxx, 1, Nx*Nx); Sxx_seq]; 
    sx_seq = [sx'; sx_seq];
end

%% forward propagation to get u
Useq = []; % Useq = [u0'; u1'; ...; uT-1']
Xseq = []; % Xseq = [x0'; x1'; ...; xT']
dxseq = [];
xt = x0;
for tt = 1:Nstep
    B = computeB(xt);
    Sxx = reshape( Sxx_seq(tt,:) , Nx, Nx);
    sx = sx_seq(tt, :)';
    % compute optimal ut given S
    [ut, Kxx, kx] = computeU(Sxx, sx, xt);
    % update x
    dxt = A*xt + B*ut + C;    
    xt = xt + dxt*tstep;
    % stack forward
    Useq = [Useq; ut'];
    Xseq = [Xseq; xt'];
    dxseq = [dxseq;dxt'];
end

%% plot
L = Xseq(:,1:3);
Pg = Xseq(:, 4:6);
dPg = Xseq(:,7:9);

dL = dxseq(:,1:3);
dPg2 = dxseq(:, 4:6);
ddPg = dxseq(:,7:9);

f1 = Useq(:,1:3);
f2 = Useq(:,4:6);
f3 = Useq(:,7:9);

figure;
subplot(3,1,1);plot(L); legend; title('L');
subplot(3,1,2);plot(Pg); legend; title('Pcom');
subplot(3,1,3);plot(dPg); legend; title('Vcom');

figure;
subplot(3,1,1);plot(dL); legend; title('dL');
subplot(3,1,2);plot(dPg2); legend; title('dPcom');
subplot(3,1,3);plot(ddPg); legend; title('dVcom');

figure;
subplot(3,1,1);plot(f1); legend; title(contactfoot{1});
subplot(3,1,2);plot(f2); title(contactfoot{2});
subplot(3,1,3);plot(f3); title(contactfoot{3});



