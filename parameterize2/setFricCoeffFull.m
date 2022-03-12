function mus = setFricCoeffFull(robot)

mus = [];
for i=1:length(robot.footname)
    fn = robot.footname{i};    
    foot = getfield(robot, fn);
    mus = [mus; foot.mu];    
end

end