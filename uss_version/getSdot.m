function [Sxxdot, sxdot, s0dot] = getSdot(Sxx, sx, Kxx, kx)

global A B C Ruu ru Qxx qx q0

Sxxdot = - ( Sxx'*(A-B*Kxx) + (A-B*Kxx)'*Sxx + Qxx + Kxx'*Ruu*Kxx );

sxdot = - ( (A-B*Kxx)'*sx + Sxx'*(-B*kx+C) + Kxx'*ru + qx + Kxx'*Ruu*kx );

s0dot = - (sx'*(-B*kx+C) + (-B*kx+C)'*sx + kx'*ru + ru'*kx + q0 + kx'*Ruu*kx);

end