function uss = getUss(robot, xgoal)
global D d Nu

% robot_full = setRobotConfig(robot, 0);
% [Pcf, Rcf, Fmf] = setConfigMatices(robot_full);
% [Af,Cf] =  setSystemMatrices(Pcf,Rcf,Fmf);
global A C
Af = A; Cf = C;

B = computeB(xgoal);
% min{uss} 1/2*||A*xg + B*uss + C||^2 = 1/2*x'*H*x?+?f'*x
% s.t. D*uss - d >=0 : A*x<=b

HH = B'*B + 1e-5*eye(Nu);
ff = B'*(Af*xgoal+Cf);
AA = -D;
bb = -d;

uss = quadprog(HH, ff, AA, bb);

%     J = 1/2*norm(A*xgoal + B*uss + C)
%     D*uss - d
end
%
% function uss = getUss(xgoal)
%     global A C D d
%     % min{uss} ||A*xg + B*uss + C||^2
%     % s.t. D*uss - d >=0
%     B = computeB(xgoal);
%     invBB = pinv(B'*B);
%     % u = u0 + 0.5*invBB*D'*lambda
%     u0 = -invBB*B'*(A*xgoal+C);
%
%     % KKT condition
%     % lambdai * ( -D*(u0 + 0.5*invBB*D'*lambda) + d )i = 0
%     % lambda > 0, D*uss-d >0
%     m = length(d);
%     for cc=0:(2^m-1)
%         % Al lambda = bl
%         ccc = dec2bin(cc);
%         Al = []; bl = [];
%         for ci = 1:m
%             if(length(ccc)<ci || ccc(ci)=='0')
%                 a = zeros(1,m);
%                 a(ci) = 1;
%                 b = 0;
%             else
%                 a = 0.5*D(ci,:)*invBB*D';
%                 b = -D(ci,:)*u0 + d(ci);
%             end
%             Al = [Al; a];
%             bl = [bl; b];
%         end
%
%         lambda = pinv(Al)*bl;
%         uss = u0 + 0.5*invBB*D'*lambda;
%         % terminate condition
%         if( checkineq(uss, D, d)) %checklambda(lambda) &&
%             display("cc = " + cc);
%             lambda
%             uss
%             D*uss-d
%            break;
%         end
%     end
% end
%
% function bcheck = checklambda(lambda)
% % return true only if all lambda>=0
% scheck = sum(lambda<0);
% bcheck = scheck==0;
% end
%
% function bcheck = checkineq(uss, D, d)
% bcheck = checklambda( D*uss-d );
% end