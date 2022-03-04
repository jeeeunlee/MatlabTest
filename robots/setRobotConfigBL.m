% motion 2 : BL ================================
% P_com(rows: 3, cols: 1)
%  0.048464   0.011378   0.114088  
% ALconfig(rows: 3, cols: 1)
%  0.095405   0.200279  -0.162506  
% ALconfig(rows: 3, cols: 3)
%  0.381896   0.382200   0.841474  
% -0.707390   0.706823   0.000002  
% -0.594773  -0.595251   0.540297  
% ARconfig(rows: 3, cols: 1)
%  0.068164  -0.187147  -0.116271  
% ARconfig(rows: 3, cols: 3)
% -0.515909   0.161221   0.841335  
% -0.298041  -0.954553   0.000157  
%  0.803125  -0.250672   0.540513  
% BLconfig(rows: 3, cols: 1)
% -0.152816   0.200451   0.224093  
% BLconfig(rows: 3, cols: 3)
%  0.382488  -0.381612   0.841472  
%  0.706273   0.707939   0.000020  
% -0.595719   0.594302   0.540300  
% BRconfig(rows: 3, cols: 1)
% -0.153434  -0.200421   0.225019  
% BRconfig(rows: 3, cols: 3)
% -0.382552  -0.381543   0.841474  
%  0.706175  -0.708037   0.000003  
%  0.595794   0.594230   0.540297  

function robot =  setRobotConfigBL(robot)
robot.swing = 'bl';
robot.pcom = [  ]';
robot.al.pos = [  ]';
robot.ar.pos = [  ]';
robot.bl.pos = [  ]';
robot.br.pos = [  ]';

robot.al.rot = [  ];
robot.ar.rot = [  ];
robot.bl.rot = [  ];
robot.br.rot = [  ];

end