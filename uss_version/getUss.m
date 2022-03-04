function uss = getUss(xgoal)
    global A C D d
    B = computeB(xgoal);
    invBB = pinv(B'*B);
    u0 = -invBB*B'*(A*xgoal+C);
    % D*(u0 + 0.5*invBB*D'*lambda) - d = 0

    lambda = pinv(0.5*D*invBB*D')*(d-D*u0);
%     lambda = (lambda>0).*lambda;
    
    lambdacheck = D*u0-d;
    for i=1:length(lambdacheck)
        if(lambdacheck(i)>0)
            lambda(i)=0;
        end
            
    end
    
    uss = u0 + 0.5*invBB*D'*lambda;
end
