%% EKF for a robot moving around landmarks
% Author: Longhao Qian
% Data: 2022 05 22
clear
load('dataset/dataset2.mat')
%%
%%%%set covariance matrices
Q=  diag([v_var om_var]);
R = diag([r_var b_var]);
T = 0.1;% sample time 0.1s
%%%%%
r_max =3;% useer defined threshold
x0_hat = x_true(1);
P0_hat = diag([1 1 0.1]);
N = max(size(x_true));
N_landmarks = zeros(N,1);
X_HAT = zeros(N,3);% estimated pose
P_hat = zeros(3*N,3);% estimated covariance
X_HAT(1,:)= [x_true(1) y_true(1) th_true(1)];
%X_HAT(1,:)= [50 50 0.4];
%P_hat(1:3,:) = P0_hat;
U = [v om];% assemble input.
Range = r;
Bearing = b;
N_break= 2900;
%%%%%%%%%%%%%%%%%%%%%%%
x_check_arry = zeros(N,3);
x_modify_arry = zeros(N,3);

%%%%%%%%%%%%%%%%%%
% manitupate input and measurement
% U = [U(1,:);U];
% Range = [Range(1,:);Range];
% Bearing = [Bearing(1,:);Bearing];
% U = [U(2:end,:);U(end,:)];
% Range = [Range(2:end,:);Range(end,:)];
% Bearing = [Bearing(2:end,:);Bearing(end,:)];
sigma3 = zeros(N,3);%3 sigma envelop
sigma3(1,:) = 3*sqrt(diag(P_hat(1:3,:)));
%%
for i = 2:N
    %determine how many land marks are seem
    LandmarkIndex = zeros(17,1);% vector that contains visible landmarks
    for j = 1:17
    if r(i,j)==0
    elseif r(i,j)>r_max
    else
        N_landmarks(i) = N_landmarks(i)+1;
        LandmarkIndex(N_landmarks(i))=j;% assign corresponding landmarks index to array
    end        
    end
    %%%%%%%%%%% estimation
    if N_landmarks(i) ==0% dead reckoning
        [Fk_1,Qk_C] = EKF_MotionJacobians(X_HAT(i-1,:)',U(i,:)',Q,T);
        % predictor:
        P_hat(3*i-2:3*i,:) = Fk_1*P_hat(3*(i-1)-2:3*(i-1),:)*Fk_1'+Qk_C;
        X_HAT(i,:) = Sys_f(X_HAT(i-1,:)',U(i,:)',zeros(2,1),T);       
    else% kalman filter
        [Fk_1,Qk_C] = EKF_MotionJacobians(X_HAT(i-1,:)',U(i,:)',Q,T);
        % predictor:
        PcheckK = Fk_1*P_hat(3*(i-1)-2:3*(i-1),:)*Fk_1'+Qk_C;
        xktu =  Sys_f(X_HAT(i-1,:)',U(i,:)',zeros(2,1),T);
        GK = zeros(2*N_landmarks(i),3);
        RK = zeros(2*N_landmarks(i),2*N_landmarks(i));
        YK = zeros(2*N_landmarks(i),1);
        G = zeros(2*N_landmarks(i),1);
        for k = 1:N_landmarks(i)% assemble observation matrix by number of landmarks seen
           [Gk,Rk_C] = EKF_ObservationJacobians(xktu,R,l(LandmarkIndex(k),1),l(LandmarkIndex(k),2),d);
           GK(2*k-1:2*k,:) = Gk;
           RK(2*k-1:2*k,2*k-1:2*k) =Rk_C;
           YK(2*k-1) = Range(i,LandmarkIndex(k));
           YK(2*k)   = Bearing(i,LandmarkIndex(k));
           G(2*k-1:2*k) = Sys_g(xktu,zeros(2,1),l(LandmarkIndex(k),1),l(LandmarkIndex(k),2),d);
        end
        % Kalman Gain
        Kk = PcheckK*GK'/(GK*PcheckK*GK'+ RK);
        P_hat(3*i-2:3*i,:) =(eye(3)-Kk*GK)*PcheckK;
        Innovation = YK-G;
        for k = 1:N_landmarks(i)
              Innovation(2*k) = atan2(sin(Innovation(2*k)),cos(Innovation(2*k)));
        end
        X_HAT(i,:) = xktu + Kk*Innovation;
    end
    X_HAT(i,3) = atan2(sin(X_HAT(i,3)),cos(X_HAT(i,3)));% convert angles into -pi-pi range
    sigma3(i,:) = 3*sqrt(diag(P_hat(3*i-2:3*i,:)));
end
%% plot
close all
figure
hold on
plot(x_true,y_true,'LineWidth',1)
plot(X_HAT(:,1),X_HAT(:,2),'-r','LineWidth',2)
plot(l(:,1),l(:,2),'ok','LineWidth',3)
grid on 
axis equal
legend('ground truth', 'estimated position')
xlabel('Position in X axis, m')
ylabel('Position in Y axis, m')
title('robot trajectory')
figure
hold on
plot(t,x_true,'LineWidth',1)
plot(t,X_HAT(:,1),'-r','LineWidth',2)
grid on
xlabel('t, s')
ylabel('Position in X axis, m')
legend('ground truth', 'estimated position')
figure
hold on
plot(t,y_true,'LineWidth',1)
plot(t,X_HAT(:,2),'-r','LineWidth',2)
grid on
xlabel('t, s')
ylabel('Position in Y axis, m')
legend('ground truth', 'estimated position')
figure
hold on
plot(t,th_true,'LineWidth',1)
plot(t,X_HAT(:,3),'-r','LineWidth',2)
grid on
xlabel('t, s')
ylabel('Yaw angle, rad')
legend('ground truth', 'estimated angle')
figure
plot(t,N_landmarks)
xlabel('t/s')
ylabel('Number of Landmarks Seen')
figure
hist(N_landmarks,0:1:12)
xlabel('Number of Landmarks Seen')
ylabel('Counting')
grid on
title('Number of visible landmarks')
figure
hold on
plot(t,X_HAT(:,1)-x_true,'-b','LineWidth',1)
plot(t(2:end),sigma3(2:end,1),'--r','LineWidth',3)
plot(t(2:end),-sigma3(2:end,1),'--g','LineWidth',3)
title('X Position Error')
ylabel('ErrX/m')
xlabel('Time/s')
legend('X Position Error','+3\sigma line','-3\sigma line')
grid on
figure
hold on
plot(t,X_HAT(:,2)-y_true,'-b','LineWidth',1)
plot(t(2:end),sigma3(2:end,2),'--r','LineWidth',3)
plot(t(2:end),-sigma3(2:end,2),'--g','LineWidth',3)
title('Y Position Error')
ylabel('ErrY/m')
xlabel('Time/s')
legend('Y Position Error','+3\sigma line','-3\sigma line')
grid on
figure
%th_err = X_HAT(:,3)-[th_true(2:end);th_true(end)];
th_err = X_HAT(:,3)-th_true;
for i = 1 : N
    err = [th_err(i) th_err(i)+2*pi th_err(i)-2*pi];
    [~,I] = min(abs(err));
            th_err(i) =  err(I);   
end
    
hold on
plot(t,th_err,'-b','LineWidth',1)
plot(t(2:end),sigma3(2:end,3),'--r','LineWidth',3)
plot(t(2:end),-sigma3(2:end,3),'--g','LineWidth',3)
title('\theta Error')
ylabel('Err_{\theta}/rad')
xlabel('Time/s')
legend('\theta Error','+3\sigma line','-3\sigma line')
grid on