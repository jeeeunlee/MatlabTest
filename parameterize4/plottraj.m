% function plottraj(p0, p1, p2, p3, v0, v1, v2, v3, Tf, Ts, Tt)

p_full = [];
v_full = [];
a_full = [];
for tt = 0:0.01:Tf
p = hermite(tt, p0, p1, v0, v1, Tf);
v = hermite1stDeriv(tt, p0, p1, v0, v1, Tf);
a = hermite2ndDeriv(tt, p0, p1, v0, v1, Tf);

p_full = [p_full,p];
v_full = [v_full,v];
a_full = [a_full,a];
end

mid_idx1 = size(p_full,2);

for tt = 0.01:0.01:(Ts+Tt)
p = hermite(tt, p1, p2, v1, v2, (Ts+Tt));
v = hermite1stDeriv(tt, p1, p2, v1, v2, (Ts+Tt));
a = hermite2ndDeriv(tt, p1, p2, v1, v2, (Ts+Tt));

p_full = [p_full,p];
v_full = [v_full,v];
a_full = [a_full,a];
end

mid_idx2 = size(p_full,2);
for tt = 0.01:0.01:Tt
p = hermite(tt, p2, p3, v2, v3, Tt);
v = hermite1stDeriv(tt, p2, p3, v2, v3, Tt);
a = hermite2ndDeriv(tt, p2, p3, v2, v3, Tt);

p_full = [p_full,p];
v_full = [v_full,v];
a_full = [a_full,a];
end


close all;


figure;
subplot(3,1,1); plot(v_full(1,:)); title("vel");
subplot(3,1,2); plot(v_full(2,:));
subplot(3,1,3); plot(v_full(3,:));

figure;
subplot(3,1,1); plot(a_full(1,:)); title("acc");
subplot(3,1,2); plot(a_full(2,:));
subplot(3,1,3); plot(a_full(3,:));

[Fc_se] = ContactForceSwingEnd(robot_goal, pcom_goal, Tt, beta, d);
[Fc_s] = ContactForceSwingGivenRatio(robot, pcom_goal, Ts+Tt, Tt, ratio, alpha, beta, d);
figure;
subplot(3,1,1);plot(Fc_s(1:3,:)');
subplot(3,1,2);plot(Fc_s(4:6,:)');
subplot(3,1,3);plot(Fc_s(7:9,:)');

% figure;
% idx_full = length(v_full);
% vnorm = zeros(idx_full,1);
% anorm = zeros(idx_full,1);
% for i=1:idx_full
%     vnorm(i) = sqrt(sum(v_full(:,i).*v_full(:,i)));
%     anorm(i) = sqrt(sum(a_full(:,i).*a_full(:,i)));
% end
% subplot(2,1,1);plot(vnorm);
% subplot(2,1,2);plot(anorm); ylim([0,3])

figure;
plot3(p_full(1,:),p_full(2,:),p_full(3,:)); hold on;
plot3(p_full(1,1),p_full(2,1),p_full(3,1),'ro');
plot3(p_full(1,mid_idx1),p_full(2,mid_idx1),p_full(3,mid_idx1),'rx');
plot3(p_full(1,mid_idx2),p_full(2,mid_idx2),p_full(3,mid_idx2),'bx');
plot3(p_full(1,end),p_full(2,end),p_full(3,end),'bx');
plot3(p_full(1,end),p_full(2,end),p_full(3,end),'bo');
grid on;
axis equal;
xlabel('x');