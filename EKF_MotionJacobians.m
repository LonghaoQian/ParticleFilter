function [Fk_1,Qk_C] = EKF_MotionJacobians(xk_1hat,uk,Q,T)
vk = uk(1);
%omegak = uk(2);
thetak_1 = xk_1hat(3);
ck = cos(thetak_1);
sk = sin(thetak_1);
Fk_1 = [1 0 -T*sk*vk;
         0 1 T*ck*vk;
         0 0 1];
Mk = T*[ck 0;
        sk 0;
        0 1];
Qk_C = Mk*Q*Mk';