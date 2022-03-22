function [DD1,DD2, dd] = findPhaseSwingEnd(robot, pcom_goal, Tt)
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

%% set inequality constraints
% D*Fc - d >= 0
[D, d] = setFrictionConeFull(robot);
% D( m*(s-1)*invA(:,1:3)*skew(g) + m*sddot*invA(:,4:6))*delP + D*invA*C - d >=0
%% method 1 : solve the problem together

dd = -D*invA*C+d;
invA1 = invA(:,1:3);
invA2 = invA(:,4:6);
% D( 0.5*m*(Tt-t)^2*invA1*skew(g) + m*invA2)*beta*pdir + D*invA*C - d >=0
% when t=0
% DD1*beta*pdir - dd >=0
DD1 = D*( 0.5*m*(Tt)^2*invA1*skew(g) + m*invA2);
% when t=Tt 
% DD2*beta*pdir - dd >=0
DD2 = D*( m*invA2);

DD = [DD1;DD2];
dd_ = [dd; dd];

end