function [Pc, Rc, Fm] = setConfigMaticesFull(robot)
Pc = []; Rc = []; Fm =[];
% robot.footname = {'ar','al','br','bl'};
for i=1:length(robot.footname)
    foot = getfield(robot, robot.footname{i});
    p = foot.pos;
    R = foot.rot;
    Pc = [Pc, skew(p)*R];
    Rc = [Rc, R];
    Fm = [Fm; 0;0;-foot.fm ];
   
end
