function [A, C] = setParameterizedSystemSwing(Pcom,Pc,Rc,Fm)
% [ pg x m*(pgddot - g) + Ldot] = PcFc + PmFm
% [      m*(pgddot - g)       ] = RcFc + RmFm

% A*Fc = S(s,ssdot)*delP + C 
global m g 
Ldot = zeros(3,1); % assume zero
A = [Pc-skew(Pcom)*Rc;Rc];
C = [ Ldot+skew(Pcom)*Rc*Fm-Pc*Fm; -m*g-Rc*Fm];

end