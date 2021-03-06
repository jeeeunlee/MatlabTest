% motion 3 : BR ================================
% P_com(rows: 3, cols: 1)
%  0.039908   0.006351   0.122016  
% ALconfig(rows: 3, cols: 1)
%  0.095899   0.200259  -0.162141  
% ALconfig(rows: 3, cols: 3)
%  0.381976   0.382162   0.841455  
% -0.707260   0.706954  -0.000017  
% -0.594876  -0.595121   0.540327  
% ARconfig(rows: 3, cols: 1)
%  0.068814  -0.186950  -0.117222  
% ARconfig(rows: 3, cols: 3)
% -0.515205   0.162913   0.841441  
% -0.301442  -0.953485   0.000036  
%  0.802307  -0.253627   0.540349  
% BLconfig(rows: 3, cols: 1)
% -0.172792   0.195263   0.259064  
% BLconfig(rows: 3, cols: 3)
%  0.174254  -0.511329   0.841533  
%  0.946657   0.322243  -0.000221  
% -0.271065   0.796682   0.540205  
% BRconfig(rows: 3, cols: 1)
% -0.152675  -0.200375   0.224966  
% BRconfig(rows: 3, cols: 3)
% -0.382525  -0.381557   0.841480  
%  0.706188  -0.708024  -0.000020  
%  0.595796   0.594236   0.540288  

function robot =  setRobotConfigBR(robot)
robot.swing = 'br';
robot.pcom = [ 0.039908   0.006351   0.122016   ]';
robot.al.pos = [  0.095899   0.200259  -0.162141   ]';
robot.ar.pos = [  0.068814  -0.186950  -0.117222 ]';
robot.bl.pos = [ -0.172792   0.195263   0.259064   ]';
robot.br.pos = [ -0.152675  -0.200375   0.224966   ]';

robot.al.rot = [  0.381976   0.382162   0.841455  
-0.707260   0.706954  -0.000017  
-0.594876  -0.595121   0.540327 ];
robot.ar.rot = [ -0.515205   0.162913   0.841441  
-0.301442  -0.953485   0.000036  
 0.802307  -0.253627   0.540349   ];
robot.bl.rot = [  0.174254  -0.511329   0.841533  
 0.946657   0.322243  -0.000221  
-0.271065   0.796682   0.540205   ];
robot.br.rot = [ -0.382525  -0.381557   0.841480  
 0.706188  -0.708024  -0.000020  
 0.595796   0.594236   0.540288   ];

end