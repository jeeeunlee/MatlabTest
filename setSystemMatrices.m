function [A,C] =  setSystemMatrices()

global Pc Rc m g Fm
A = [ zeros(3) skew(Rc*Fm) zeros(3);
    zeros(3) zeros(3) eye(3);
    zeros(3,9)];

C = [Pc*Fm; zeros(3,1); Rc*Fm/m + g];
end