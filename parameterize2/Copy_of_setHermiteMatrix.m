function S = setHermiteMatrix(s, sddot, Pcom)
% [ pg x m*(pgddot - g) + Ldot] = PcFc + PmFm
% [      m*(pgddot - g)       ] = RcFc + RmFm

% where, p(t) = Pcom + delP(2s -1 ), pddot(t) = 2delP*sddot
% A*Fc + S(s,ssdot)*delP + C = 0
global m g
S = - [skew(2*sddot*Pcom + (2*s-1)*g); 2*m*sddot*eye(3)];

end