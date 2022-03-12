
function [Uf, uu] = buildUf(mu)
mu = mu / sqrt(2);
Uf = [0,0,1;
    1,0,mu;
    -1,0,mu;
    0,1,mu;
    0,-1,mu];
uu = zeros(5,1);
end

% function [Uf, uu] = buildUf(mu)
% mu = mu / sqrt(2);
% Uf = [0,0,1;
%     1,0,mu;
%     -1,0,mu;
%     0,1,mu;
%     0,-1,mu];
% 
% uu = 5*ones(5,1);
% end

% function [Uf, uu] = buildUf(mu)
% mu = mu / sqrt(2);
% Uf = [0,0,-1;
%     0,0,1;
%     1,0,mu;
%     -1,0,mu;
%     0,1,mu;
%     0,-1,mu];
% maxfz = 100;
% uu = [ -maxfz ;1*ones(5,1)];
% end