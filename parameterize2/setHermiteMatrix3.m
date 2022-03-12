function S = setHermiteMatrix3(t, T2, P2)
% [ pg x m*(pgddot - g) + Ldot] = PcFc + PmFm
% [      m*(pgddot - g)       ] = RcFc + RmFm


% A*Fc = S(s,ssdot)*delP + C
s=t/T2;
global m g
S =  [-2*m/T2/T2*skew(P2) + m*(2*s-s*s)*skew(g); -2*m/T2/T2*eye(3)];

end