function [Sxxdot, sxdot, s0dot] = getSdot(Sxx, sx, Kxx, kx)

global A C R Qxx qx q0

Sxxdot = - ( 2*Sxx'*(A-B*Kxx) + Qxx + Kxx'*R*Kxx );

sxdot1 = - ( 2*Sxx'*(-B*kx+C) + qx + Kxx'*R*kx );
sxdot2 = - ( 2*sx'*(A-B*Kxx)+qx'+ kx'*R*Kxx );
sxdot = 0.5*(sxdot1 + sxdot2);

s0dot = - (2*sx'*(-B*kx+C) + q0 + kx'*R*kx);

end