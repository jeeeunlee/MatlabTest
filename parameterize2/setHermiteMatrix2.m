function S = setHermiteMatrix2(t, T1, T2, P2, alpha)
% [ pg x m*(pgddot - g) + Ldot] = PcFc + PmFm
% [      m*(pgddot - g)       ] = RcFc + RmFm


% A*Fc = S(s,ssdot)*delP + C
s=t/T1;
global m g
S = m*[ (alpha*(-2+2*s)*skew(g)-alpha*2/T1/T1*skew(P2)+T1/T2*(2*s*s-2*s)*skew(g)+4/T1/T2*skew(P2));
    -alpha*2/T1/T1*eye(3)+4/T1/T2*eye(3) ];

end