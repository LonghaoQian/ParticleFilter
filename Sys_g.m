% nonlinear observation model
function yk = Sys_g(xktu,nk,xl,yl,d)
xk = xktu(1);
yk=  xktu(2);
thetak = xktu(3);
ck = cos(thetak);
sk = sin(thetak);
t2 = xk-xl+d*ck;
t3 = yk-yl+d*sk;
yk = [sqrt(t2^2+t3^2);atan2(-t3,-t2)-thetak]+nk;
%yk = [sqrt(t2^2+t3^2);atan2(sk*t2-ck*t3,-ck*t2-sk*t3)]+nk;