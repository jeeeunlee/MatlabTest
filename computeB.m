function B = computeB(x)
global Pc Rc m Nu
pg = x(4:6);
B = [Pc-skew(pg)*Rc;zeros(3, Nu);Rc/m];

end