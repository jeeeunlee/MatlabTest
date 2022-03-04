function [Sxx_seq, sx_seq] = bwd_prop(x0)
global A C B D invR tstep Nstep Nx
B = computeB(x0);
Sxx_seq = [];
sx_seq = [];

% Sxx = pinv(B*invR*B')*A;
% sx = pinv(B*invR*B')*C;
% err = 1;
% while (err>1e-3)
%     [lambda, Lxx, lx]  = computeLambda(Sxx, sx, x0);
%     sx_ = pinv(B*invR*B')*(C+0.5*B*invR*D'*lambda);
%     err = norm(sx-sx_);
%     sx = sx_;    
% end

wL = 1;
wP = 1;
wdP = 10;
Sxx = diag([wL,wL,wL, wP,wP,wP, wdP,wdP,wdP]);
sx = -Sxx*x0;

Sxx_seq = [reshape(Sxx, 1, Nx*Nx); Sxx_seq]; 
sx_seq = [sx'; sx_seq];
    
for tt = 1:Nstep
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