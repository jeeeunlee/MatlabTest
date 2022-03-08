function [Sxx_seq, sx_seq] = bwd_prop(x0)
global B tstep Nstep Nx
% global A C D invR
B = computeB(x0);
Sxx_seq = [];
sx_seq = [];

% final state soft constraint
wL = 1;
wP = 1;
wdP = 100;
Sxx = diag([wL,wL,wL, wP,wP,wP, wdP,wdP,wdP]);
sx = -Sxx*x0;

Sxx_seq = [reshape(Sxx, 1, Nx*Nx); Sxx_seq]; 
sx_seq = [sx'; sx_seq];
    
for tt = 1:(Nstep-1)
    [u, Kxx, kx] = computeU(Sxx, sx, x0);
    [Sxxdot, sxdot, s0dot] = getSdot(Sxx, sx, Kxx, kx);
    
%     xdot = A*xgoal + B*u + C;
%     ddpg = 1/m*(Rc*(u+Fm))+g;
    
    Sxx = Sxx - Sxxdot*tstep;
    sx = sx - sxdot*tstep;
    % stack backward
    Sxx_seq = [reshape(Sxx, 1, Nx*Nx); Sxx_seq]; 
    sx_seq = [sx'; sx_seq];
end