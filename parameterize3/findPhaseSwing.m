function [delP, sddot] = findPhaseSwing(robot, pcom_goal, Ts)
global m g Nx Nu
%% get robot system matrices
% A*Fc = S(s,sddot)*delP + C
% A1*Fc = m*(s-1)*skew(g)*delP + C1
% A2*Fc = m*sddot*delP + C2 

[Pc, Rc, Fm] = setConfigMatices(robot);
[A, C] = setParameterizedSystemSwing(pcom_goal,Pc,Rc,Fm);
A1 = A(1:3,:);
A2 = A(4:6,:);
C1 = C(1:3);
C2 = C(4:6);


%% set inequality constraints
% D*Fc - d >= 0
[D, d] = setFrictionCone(robot);
mu_ = setFricCoeff(robot);


%%

% given W
w=[];
for i=1:length(mu_)
    w = [w; 1./mu_(i);1./mu_(i);mu_(i)];
end
W = diag(w);
invW = diag(1./w);
invA = invW*A'*pinv(A*invW*A');

% D( m*(s-1)*invA(:,1:3)*skew(g) + m*sddot*invA(:,4:6))*delP + D*invA*C - d >=0
%% method 1 : solve the problem together
Ts = 0.35;
dd = -D*invA*C+d;
sddot = -2/Ts/Ts;
% when s=0 
% D( m*(-1)*invA(:,1:3)*skew(g) + m*sddot*invA(:,4:6))*delP + D*invA*C - d >=0
% DD1*delP - dd >=0
DD1 = D*( m*(-1)*invA(:,1:3)*skew(g) + m*sddot*invA(:,4:6));
% when s=1 
% D( m*sddot*invA(:,4:6))*delP + D*invA*C - d >=0
DD2 = D*( m*sddot*invA(:,4:6));

DD = [DD1;DD2];
dd = [dd; dd];

delP = quadprog(eye(3),zeros(3,1),-DD,-dd);


end