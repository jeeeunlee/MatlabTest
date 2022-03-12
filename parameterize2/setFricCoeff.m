function mus = setFricCoeff(robot)

mus = [];
for i=1:length(robot.footname)
    fn = robot.footname{i};
    if(~strcmp(robot.swing,fn))
        foot = getfield(robot, fn);
        mus = [mus; foot.mu];
    end
end

end