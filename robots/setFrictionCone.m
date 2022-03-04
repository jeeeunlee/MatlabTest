%% Ku*u - k >= 0

function [Ku, k] = setFrictionCone(robot)
Ku = []; k=[];
% robot.footname = {'ar','al','br','bl'};
for i=1:length(robot.footname)
    fn = robot.footname{i};
    if(~strcmp(robot.swing,fn))
        foot = getfield(robot, fn);
        [Uf, uu] = buildUf(foot.mu);
        [cs,rs] = size(Ku);
        [cu,ru] = size(Uf);
        Ku = [Ku zeros(cs,ru);zeros(cu,rs) Uf];
        k = [k; uu];
    end
end
end

function [Uf, uu] = buildUf(mu)
mu = mu / sqrt(2);
Uf = [0,0,1;
    1,0,mu;
    -1,0,mu;
    0,1,mu;
    0,-1,mu];
uu = zeros(5,1);
end