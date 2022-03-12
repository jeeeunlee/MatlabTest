function [DD, dd] = findPhase3(robot)
global m g Nx Nu
%% get robot system matrices
% A*Fc = S(s,ssdot)*delP + C

% [Pc, Rc, Fm] = setConfigMatices(robot);
[Pc, Rc, Fm] = setConfigMaticesFull(robot);
[A, C] = setParameterizedSystem3(robot.pcom,Pc,Rc,Fm);

%% set inequality constraints
% D*Fc - d >= 0
% [D, d] = setFrictionCone(robot);
[D, d] = setFrictionConeFull(robot);
% mu_ = setFricCoeff(robot);
mu_ = setFricCoeffFull(robot);

%%
% given W, S
% find delP s.d. -D*invA*(S*delP+C)-d>0
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
T2 = 0.5; timestep=0.01;

% find delP s.d. -D*invA*(S*delP+C)-d>0
% DD*delP + dd >=0
dd = D*invA*C - d;


for t=0:timestep:T2
    S = setHermiteMatrix3(t, T2, robot.pcom);
    DD = D*invA*S;
    
    quadprog(eye(3),zeros(3,1),-DD,dd)
end

end