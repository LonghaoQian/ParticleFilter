function [Gk,Rk_C] = EKF_ObservationJacobians(xktu,R,xl,yl,d)
xk = xktu(1);
yk=  xktu(2);
thetak = xktu(3);
ck = cos(thetak);
sk = sin(thetak);
t2 = xk-xl+d*ck;
t3 = yk-yl+d*sk;
t4 = -d*sk*t2+d*ck*t3;
t5 = t2*t2+t3*t3;
t1 = sqrt(t5);
Gk = [t2/t1 t3/t1 t4/t1;
      -t3/t5 t2/t5 d*(t3*sk+t2*ck)/t5-1];
Nk = eye(2);
Rk_C = Nk*R*Nk';