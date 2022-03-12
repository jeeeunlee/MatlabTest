function [A, C] = setParameterizedSystem(Pcom,Pc,Rc,Fm)
% [ pg x m*(pgddot - g) + Ldot] = PcFc + PmFm
% [      m*(pgddot - g)       ] = RcFc + RmFm

% where, p(t) = Pcom + (2s -1)*delP, pddot(t) = 2*sddot*delP
% A*Fc + S(s,ssdot)*delP + C = 0

global m g 
Ldot = zeros(3,1); % assume zero
A = [Pc;Rc];
C = [Pc*Fm + m*skew(Pcom)*g - Ldot; Rc*Fm + m*g];

end
