function [Pc, Rc, Fm] = setConfigMatices(robot)
Pc = []; Rc = []; Fm =[];
% robot.footname = {'ar','al','br','bl'};
for i=1:length(robot.footname)
    fn = robot.footname{i};
    if(~strcmp(robot.swing,fn))
        foot = getfield(robot, fn);
        p = foot.pos;
        R = foot.rot;
        Pc = [Pc, skew(p)*R];
        Rc = [Rc, R];
        Fm = [Fm; 0;0;-foot.fm ];
    end
end
