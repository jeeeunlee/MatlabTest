function [A,C] =  setSystemMatrices(Pc,Rc,Fm)

global  m g 
A = [ zeros(3) skew(Rc*Fm) zeros(3);
    zeros(3) zeros(3) eye(3);
    zeros(3,9)];

C = [Pc*Fm; zeros(3,1); Rc*Fm/m + g];
end