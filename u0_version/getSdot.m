function [Sxxdot, sxdot, s0dot] = getSdot(Sxx, sx, Kxx, kx)

global A B C R Qxx qx q0

Sxxdot = - ( Sxx'*(A-B*Kxx) + (A-B*Kxx)'*Sxx + Qxx + Kxx'*R*Kxx );

% sxdot1 = - ( 2*Sxx'*(-B*kx+C) + qx + Kxx'*R*kx );
% sxdot2 = - ( 2*sx'*(A-B*Kxx)+qx'+ kx'*R*Kxx );
% sxdot = 0.5*(sxdot1 + sxdot2');
sxdot = - ( (A-B*Kxx)'*sx + Sxx'*(-B*kx+C) + qx + Kxx'*R*kx );

s0dot = - (sx'*(-B*kx+C) + (-B*kx+C)'*sx + q0 + kx'*R*kx);

end