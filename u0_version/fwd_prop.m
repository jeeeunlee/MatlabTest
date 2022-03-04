function [Useq, Xseq, dxseq] = fwd_prop(x0, Sxx_seq, sx_seq)

global A C B Nx tstep Nstep
Useq = []; % Useq = [u0'; u1'; ...; uT-1']
Xseq = []; % Xseq = [x0'; x1'; ...; xT']
dxseq = [];
xt = x0;
for tt = 1:Nstep
    B = computeB(xt);
    Sxx = reshape( Sxx_seq(tt,:) , Nx, Nx);
    sx = sx_seq(tt, :)';
    % compute optimal ut given S
    [ut, Kxx, kx] = computeU(Sxx, sx, xt);
    % update x
    dxt = A*xt + B*ut + C;    
    xt = xt + dxt*tstep;
    % stack forward
    Useq = [Useq; ut'];
    Xseq = [Xseq; xt'];
    dxseq = [dxseq;dxt'];
end
end

% for tt = Nstep:-1:1
%     B = computeB(xt);
%     Sxx = reshape( Sxx_seq(tt,:) , Nx, Nx);
%     sx = sx_seq(tt, :)';
%     % compute optimal ut given S
%     [ut, Kxx, kx] = computeU(Sxx, sx, xt);
%     % update x
%     dxt = A*xt + B*ut + C;
%     xt = (eye(Nx)+tstep*A-tstep*B*Kxx)\(xt+tstep*B*kx-C*tstep);
%     % stack forward
%     Useq = [ut';Useq];
%     Xseq = [xt';Xseq];
%     dxseq = [dxt';dxseq];
% end