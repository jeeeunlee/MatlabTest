% motion 4 : AL ================================
% P_com(rows: 3, cols: 1)
%  0.041396   0.000354   0.129147  
% ALconfig(rows: 3, cols: 1)
%  0.096608   0.199881  -0.162031  
% ALconfig(rows: 3, cols: 3)
%  0.381096   0.381479   0.842164  
% -0.708712   0.705497   0.001133  
% -0.593712  -0.597283   0.539221  
% ARconfig(rows: 3, cols: 1)
%  0.073275  -0.186805  -0.124219  
% ARconfig(rows: 3, cols: 3)
% -0.517103   0.157426   0.841321  
% -0.291033  -0.956713   0.000140  
%  0.804925  -0.244780   0.540536  
% BLconfig(rows: 3, cols: 1)
% -0.172284   0.195242   0.258232  
% BLconfig(rows: 3, cols: 3)
%  0.173344  -0.511530   0.841599  
%  0.947062   0.321050   0.000070  
% -0.270231   0.797035   0.540103  
% BRconfig(rows: 3, cols: 1)
% -0.168493  -0.207559   0.252304  
% BRconfig(rows: 3, cols: 3)
% -0.264888  -0.469220   0.842418  
%  0.871379  -0.490610   0.000729  
%  0.412956   0.734258   0.538825  
% ================================

function robot =  setRobotConfigAL(robot)
robot.swing = 'al';
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