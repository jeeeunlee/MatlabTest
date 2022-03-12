function [A, C] = setParameterizedSystem2(Pcom,Pc,Rc,Fm)
% [ pg x m*(pgddot - g) + Ldot] = PcFc + PmFm
% [      m*(pgddot - g)       ] = RcFc + RmFm

% A*Fc = S(s,ssdot)*delP + C 
global m g 
Ldot = zeros(3,1); % assume zero
A = [Pc;Rc];
C = [ - m*skew(Pcom)*g+Ldot-Pc*Fm; -m*g-Rc*Fm];

end