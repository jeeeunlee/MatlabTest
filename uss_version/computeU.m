function [u, Kxx, kx] = computeU(Sxx, sx, x)

global invR D B ru

% global Pc Rc m Nu


% [Lxx lx]*[x;1];
[lambda, Lxx, lx]  = computeLambda(Sxx, sx, x);
% u = invR*(0.5*D'*lambda - B'*(Sxx*x+sx) - ru);
% u = invR*(0.5*D'*(Lxx*x+lx) - B'*(Sxx*x+sx) -ru);
% Kxx = - (0.5*invR*D'*Lxx - invR*B'*Sxx);
% kx = -(0.5*invR*D'*lx - invR*(B'*sx+ru));
Kxx = - ( - invR*B'*Sxx);
kx = -(0.5*invR*D'*lambda - invR*(B'*sx+ru));

u = -[Kxx kx]*[x;1];
end