%% Ku*u - k >= 0

function [Ku, k] = setFrictionConeFull(robot)
Ku = []; k=[];
% robot.footname = {'ar','al','br','bl'};
for i=1:length(robot.footname)
    fn = robot.footname{i};
    foot = getfield(robot, fn);
    [Uf, uu] = buildUf(foot.mu);
    [cs,rs] = size(Ku);
    [cu,ru] = size(Uf);
    Ku = [Ku zeros(cs,ru);zeros(cu,rs) Uf];
    k = [k; uu];
end
end
