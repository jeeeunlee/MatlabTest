function [DDas, DDbs, dds] = findPhaseSwingGivenRatio(robot, pcom_goal, T2, T3, ratio)
%% alpha = ratio * beta
global m g
%% get robot system matrices
% A*Fc = Ba(t)*alpha*pdir + Bb(t)*beta*pdir + C
% A1*Fc = ( 0.5*m*skew(g)*(T2-t)^2 )*alpha*pdir + ( 0.5*m*skew(g)*T3*(T3+2*T2-t)*beta*pdir + C1
% A2*Fc = ( m )*alpha*pdir + ( 0 )*beta*pdir + C2 

[Pc, Rc, Fm] = setConfigMatices(robot);
[A, C] = setParameterizedSystemSwing(pcom_goal,Pc,Rc,Fm);

%% weighted inverse
mu_ = setFricCoeff(robot);
w=[];
for i=1:length(mu_)
    w = [w; 1./mu_(i);1./mu_(i);mu_(i)];
end
W = diag(w);
invW = diag(1./w);
invA = invW*A'*pinv(A*invW*A');

%% set inequality constraints
% D*Fc - d >= 0
[D, d] = setFrictionCone(robot);

%% method 1 : solve the problem together
% Fc = invA*( Ba(t)*alpha*pdir + Bb(t)*beta*pdir + C )
% D*( ( 0.5*m*(T2-t)^2*invA1*skew(g) + m*invA2 )*alpha*pdir 
%      + ( 0.5*m*T3*(T3+2*T2-t)*invA1*skew(g) )*beta*pdir ) + C1 > d

dd = -D*invA*C+d;
invA1 = invA(:,1:3);
invA2 = invA(:,4:6);

% when t=0 
DDa1 = DDa(0);
DDb1 = DDb(0);

% when t=T2
DDa2 = DDa(T2);
DDb2 = DDb(T2);
DDas = [DDa1; DDa2];
DDbs = [DDb1; DDb2];
dds = [dd;dd];

% when t=local minima
% alpha = ratio*beta
% g(t) = ratio*(T2-t)^2 + T3*(T3+2T2-t)
t_star = T2 + T3/2/ratio;
if(t_star>0 && t_star<T2)
    DDas = [DDas; DDa(t_star)];
    DDbs = [DDbs; DDb(t_star)];
    dds = [dds; dd];
end

% DD = [DD1;DD2];

    function Da = DDa(t)
        Da =  D*( 0.5*m*(T2-t)^2*invA1*skew(g) + m*invA2 );
    end

    function Db = DDb(t)
        Db = D*( 0.5*m*T3*(T3+2*T2-t)*invA1*skew(g) );
    end


end

