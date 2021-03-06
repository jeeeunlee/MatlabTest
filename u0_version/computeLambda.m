function [lambda, Lxx, lx] = computeLambda(Sxx, sx, x)

global D d invR B Nx

% lambda0 = (0.5*D*invR*D')\(d+D*invR*B'*(Sxx*x+sx));
Lxx = pinv(0.5*D*invR*D')*(D*invR*B'*Sxx);
lx = pinv(0.5*D*invR*D')*(d+D*invR*B'*sx );
lambda0 = [Lxx lx]*[x;1];

lambda=lambda0;
for i=1:length(lambda0)
    if(lambda0(i)<0)
        lambda(i) = 0;
        Lxx(i,:)=zeros(1,Nx);
        lx(i) = 0;
    end    
end
% lambda = (lambda0>0).*lambda0;
end