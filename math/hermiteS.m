function [s_, sdot_, sddot_] = hermiteS(T, timestep)

p1 = 0;
p2 = 1;
v1 = 0;
v2 = 0;

s_ = [];
sdot_=[];
sddot_=[];
for tt = 0:timestep:T
    s = hermite(tt, p1,p2,v1,v2,T);
    sdot = hermite1stDeriv(tt, p1, p2, v1, v2, T);
    sddot = hermite2ndDeriv(tt, p1, p2, v1, v2, T);
    s_ = [s_;s];
    sdot_ = [sdot_; sdot];
    sddot_ = [sddot_; sddot];
    
end
% figure; subplot(3,1,1); plot(s_);
% subplot(3,1,2); plot(sdot_);
% subplot(3,1,3); plot(sddot_);