function [u, Kxx, kx] = computeU(Sxx, sx, x)

global invR D B

% global Pc Rc m Nu


% [Lxx lx]*[x;1];
[lambda, Lxx, lx]  = computeLambda(Sxx, sx, x);
% u = invR*(0.5*D'*lambda - B'*(Sxx*x+sx));
% u = invR*(0.5*D'*(Lxx*x+lx) - B'*(Sxx*x+sx));
Kxx = - (0.5*invR*D'*Lxx - invR*B'*Sxx);
kx = -(0.5*invR*D'*lx - invR*B'*sx);
u = -[Kxx kx]*[x;1];
end