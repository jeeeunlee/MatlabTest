function [Fc] = ContactForceSwingEnd(robot, pcom_goal, Tt, beta, pdir)
global m g
%% get robot system matrices
% Let d2p/dt2 = beta*pdir;
% A*Fc = B(t)*beta*pdir + C
% A1*Fc = 0.5*m*(Tt-t)^2*skew(g)*beta*pdir + C1
% A2*Fc = m*beta*pdir + C2 

[Pc, Rc, Fm] = setConfigMaticesFull(robot);
[A, C] = setParameterizedSystemSwingEnd(pcom_goal,Pc,Rc,Fm);

%% Weighted inverse
mu_ = setFricCoeffFull(robot);
w=[];
for i=1:length(mu_)
    w = [w; 1./mu_(i);1./mu_(i);mu_(i)];
end
W = diag(w);
invW = diag(1./w);
invA = invW*A'*pinv(A*invW*A');

invA1 = invA(:,1:3);
invA2 = invA(:,4:6);
Fc=[];
for t=0:0.01:Tt
Fc = [Fc, ( 0.5*m*(Tt-t)^2*invA1*skew(g) + m*invA2)*beta*pdir + invA*C];
end
end