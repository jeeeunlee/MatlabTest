function [A, C] = setParameterizedSystem(Pcom,Pc,Rc,Fm)
% [ pg x m*(pgddot - g) + Ldot] = PcFc + PmFm
% [      m*(pgddot - g)       ] = RcFc + RmFm

% where, p(t) = Pcom + delP(2s -1 ), pddot(t) = 2delP*sddot
% A*Fc + S(s,ssdot)*delP + C = 0

global m g 
Ldot = zeros(3,1); % assume zero
A = [Pc;Rc];
C = [m*skew(Pcom)*g - Ldot + Pc*Fm; Rc*Fm];

end