% motion 1 : AR ================================
% motion 2 : BL ================================
% motion 3 : BR ================================
% motion 4 : AL ================================


function robot =  setRobotConfig(robot, motion)
% assume no residual magnetic force in this case
robot.al.fm = 70; robot.al.mu = 0.5;
robot.ar.fm = 40; robot.ar.mu = 0.3;
robot.bl.fm = 70; robot.bl.mu = 0.5;
robot.br.fm = 70; robot.br.mu = 0.5;
switch(motion)
    case 1
        robot = setRobotConfigAR(robot);%
    case 2
        robot = setRobotConfigBL(robot);
    case 3
        robot = setRobotConfigBR(robot);%
    case 4
        robot = setRobotConfigAL(robot);
    case 0
        robot = setRobotConfigFull(robot);
end