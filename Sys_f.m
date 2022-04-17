% nonlinear motion model
function xk = Sys_f(xk_1,uk,wk,T)
theta_k = xk_1(3);
ck = cos(theta_k);
sk = sin(theta_k);
xk = xk_1+T*[ck 0;sk 0;0 1]*(uk+wk);
