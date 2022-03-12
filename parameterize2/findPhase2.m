function [DD, dd] = findPhase2(robot)
global m g Nx Nu
%% get robot system matrices
% A*Fc = S(s,ssdot)*delP + C

[Pc, Rc, Fm] = setConfigMatices(robot);
[A, C] = setParameterizedSystem2(robot.pcom,Pc,Rc,Fm);

%% set inequality constraints
% D*Fc - d >= 0
[D, d] = setFrictionCone(robot);
mu_ = setFricCoeff(robot);


%%
% given W, S
% find delP s.d. D*invA*(S*delP+C)-d>0
% DD*delP - dd >=0

% given W
w=[];
for i=1:length(mu_)
    w = [w; 1./mu_(i);1./mu_(i);mu_(i)];
end
W = diag(w);
invW = diag(1./w);
invA = invW*A'*pinv(A*invW*A');

% given S
T1 = 0.4; timestep=0.1;
T2 = 0.4;
alpha=1;

% find delP s.t. -D*invA*(S*delP+C)-d>0
% DD*delP + dd >=0
dd = D*invA*C - d;


for t=0:timestep:T1
    S = setHermiteMatrix2(t, T1, T2, robot.pcom, alpha);
    DD = D*invA*S;
    
    quadprog(eye(3),zeros(3,1),-DD,dd)
end

end