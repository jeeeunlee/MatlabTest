function R = setWeightMatrices(robot)
w = [];
for i=1:length(robot.footname)
    fn = robot.footname{i};
    if(~strcmp(robot.swing,fn))
        foot = getfield(robot, fn);
%         wx = 1/foot.mu;
%         wz = 1; %foot.mu;
        wx = 1;
        wz = 2*(foot.mu^2+1);
        w = [w; wx; wx; wz];
    end
end

R = diag(w);

end