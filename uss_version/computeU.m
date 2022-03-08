function [u, Kxx, kx] = computeU(Sxx, sx, x)

global invR D B Ruu ru d

%% min H(u) = (x'*Sxx +sx')*(B*u)+0.5*(u'Ruu*u + 2ru'*u) + Constant(x)
% s.t. Du - d > 0
% f(x) = 1/2*x'*H*x?+?f'*x, Ax<b
HH = Ruu;
ff = ru + B'*(Sxx*x+sx);
AA = -D;
bb = -d;
[uu,fval,exitflag,output,lambda] = quadprog(HH, ff, AA, bb);

% u = invR*(0.5*D'*lambda - B'*(Sxx*x+sx) - ru);
% u = invR*(0.5*D'*(Lxx*x+lx) - B'*(Sxx*x+sx) -ru);

Kxx = - ( - invR*B'*Sxx);
kx = -(0.5*invR*D'*2*lambda.ineqlin - invR*(B'*sx+ru));

u = -[Kxx kx]*[x;1];
end


% function [u, Kxx, kx] = computeU(Sxx, sx, x)
% 
% global invR D B ru
% 
% % global Pc Rc m Nu
% 
% 
% % [Lxx lx]*[x;1];
% [lambda, Lxx, lx]  = computeLambda(Sxx, sx, x);
% % u = invR*(0.5*D'*lambda - B'*(Sxx*x+sx) - ru);
% % u = invR*(0.5*D'*(Lxx*x+lx) - B'*(Sxx*x+sx) -ru);
% % Kxx = - (0.5*invR*D'*Lxx - invR*B'*Sxx);
% % kx = -(0.5*invR*D'*lx - invR*(B'*sx+ru));
% Kxx = - ( - invR*B'*Sxx);
% kx = -(0.5*invR*D'*lambda - invR*(B'*sx+ru));
% 
% u = -[Kxx kx]*[x;1];
% end