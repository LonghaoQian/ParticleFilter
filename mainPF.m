%% robot trajectory estimation using particle filter
% author: Longhao Qian
% date: 2022 05 22
clear
load ('dataset/dataset2.mat')
%% set covariance matrices
Q = diag([v_var om_var]);
R = diag([r_var b_var]);
T = 0.1;% sample time 0.1s
%% initialization
r_max =3;% useer defined threshold
x0_hat = x_true(1);
P0_hat = diag([1 1 0.1]);
N = max(size(x_true));
N_landmarks = zeros(N,1);
N_end = 3000;
%% system model
x_hat = zeros(N, 3); % estimated pose
P_hat = zeros(3 * N, 3); % estimated covariance
x_hat(1,:) = [x_true(1) y_true(1) th_true(1)];
sigma3 = zeros(N,3); % 3 sigma envelop
sigma3(1,:) = 3*sqrt(diag(P_hat(1:3,:)));
U = [v om];% assemble input.
Range = r;
Bearing = b;
N_break= 2900;
%% particle filter parameter
Ns = 200;                                % number of particles
nx = 3;                                  % number of states
particles = zeros(nx, Ns, N);
weights = zeros(Ns, N);
resample_percentaje = 0.25;
Nt = resample_percentaje * Ns;
for i = 1 : Ns                                  % simulate initial particles
      particles(1, i, 1) = -4 + 16 * rand(1, 1);     % at time k=1
      particles(2, i, 1) = -5 + 10 * rand(1, 1);
      particles(3, i, 1) = -pi + 2 * pi * rand(1, 1);
end   
weights(:, 1) = repmat(1 / Ns, Ns, 1);           % all particles have the same weight
%% define video settings
anim1 = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.7]);
ax = axes('Position',[0.08 0.05 0.90 0.90],'XLim',[-3 12],'YLim',[-5 6],...
    'ZLim',[-1 1],'DataAspectRatioMode','manual'); % axis for the 
anim1.Renderer='Painters';
% define triangle symbol
l_symbol = 0.2;
h_symbol = 0.4;
faceCoordinates = [h_symbol * 2 /3, 0, 0;
                  -h_symbol * 1 /3, l_symbol / 2, 0;
                  -h_symbol * 1 /3, -l_symbol / 2, 0];
vertices_index = [1, 2, 3];
for i = 1 : Ns
    symbolBojAx{i} = hgtransform('Parent',ax);
    symbolBoj{i} = patch('Faces', vertices_index, 'Vertices', faceCoordinates, 'FaceColor','cyan');
    set(symbolBoj{i},'Parent', symbolBojAx{i});
end
symbolBojAx{Ns + 1} = hgtransform('Parent',ax);
symbolBoj{Ns + 1} = patch('Faces', vertices_index, 'Vertices', faceCoordinates, 'FaceColor','blue');
set(symbolBoj{Ns + 1},'Parent', symbolBojAx{Ns + 1});
% define the landmarks
N_gamma = 20;
gamma = linspace(-pi, pi, N_gamma);
radius = 0.1;
[di1, di2] = size(l);
landmarkCoordinates = zeros(di1, 3);
for i = 1 : N_gamma
    landmarkCoordinates(i, :) = radius * [cos(gamma(i)), sin(gamma(i)), 0];
end
for i = 1 : di1
    offSet = [ones(N_gamma, 1) * l(i, 1) ones(N_gamma, 1) * l(i, 2) zeros(N_gamma, 1)];
    landmarkSymbol{i} = patch('Faces', 1:1:20 , 'Vertices', landmarkCoordinates + offSet, 'FaceColor','black');
end
grid on 
axis equal
xlabel('Position X/m')
ylabel('Position Y/m')
%% attach the symbol to histogram
TR = GetTransformMatrix(x_true(1), y_true(1), th_true(1));
set(symbolBojAx{Ns + 1},'Matrix',TR);
for k = 1 : Ns
    TR = GetTransformMatrix(particles(1, k, 1),  particles(2, k, 1),  particles(3, k, 1));
    set(symbolBojAx{k},'Matrix',TR);
end
drawnow;
%% run the estimator
for i = 2 : N_end
    %% determine how many land marks are seem
    LandmarkIndex = zeros(17, 1);% vector that contains visible landmarks
    for j = 1 : 17
        if r(i, j) == 0
        elseif r(i, j) > r_max
        else
            N_landmarks(i) = N_landmarks(i) + 1;
            LandmarkIndex(N_landmarks(i)) = j; % assign corresponding landmarks index to array
        end
    end
    %% draw samples from prior belief
    xkm1 = particles(: , :, i - 1); % extract particles from last iteration;
    xk = zeros(nx, Ns);
    for j = 1 : Ns
        xk(:, j) = Sys_f(xkm1(:, j), U(i, :)', normrnd(zeros(2, 1), 10 * [v_var om_var]'), T); % draw samples
        % wrap the angle
        xk(3, j) = WrapAnlge(xk(3, j));
    end
    %% 
    if N_landmarks(i) ~= 0
        % perform observation test
        %% get the observation model
        % p_yk_given_xk = @(k, yk, xk) p_obs_noise(yk - obs(k, xk, 0));
        yk = zeros(2 * N_landmarks(i), 1);
        for k = 1 : N_landmarks(i) 
            % assemble observation matrix by number of landmarks seen
            yk(2*k-1) = Range(i, LandmarkIndex(k));
            yk(2*k)   = Bearing(i, LandmarkIndex(k));
        end
        %% update weights
        for j = 1 : Ns
            % the observation noise 
            weights(j, i) = weights(j, i - 1) * GetObservationSample(xk(:, j), yk, 5* R, N_landmarks(i), l, LandmarkIndex, d); 
        end
        %% Normalize weight vector
        weights(:, i) = weights(:, i)./sum(weights(:, i));
        [V, Imax] = max(weights(:, i));
        %% Calculate effective sample size: eq 48, Ref 1
        Neff = 1/sum(weights(:, i).^2);
        %% Resampling
        % remove this condition and sample on each iteration:
        % [xk, wk] = resample(xk, wk, resampling_strategy);
        %if you want to implement the bootstrap particle filter
        if Neff < Nt
          % disp('Resampling ...')
          [xk_temp,  weights(:, i)] = ResampleParticle(xk, weights(:, i), 'systematic_resampling');
          % {xk, wk} is an approximate discrete representation of p(x_k | y_{1:k})
          xk = xk_temp;
        end
    else
        % if the number of landmarks is zero, keep the value of the weights  
        weights(:, i) = weights(:, i - 1);
    end 
    %% Compute estimated state
    % pick the particle with the largest weight as the estimation
    [~, Iest] = max(weights(:, i));
    x_hat(i, :) = xk(:, Iest)';
    %% Store new weights and particles
    particles(: , :, i) = xk;
    %% show the points
    % change the particle position and ground truth
    TR = GetTransformMatrix(x_true(i), y_true(i), th_true(i));
    set(symbolBojAx{Ns + 1},'Matrix',TR);
    for k = 1 : Ns
        TR = GetTransformMatrix(xk(1, k), xk(2, k), xk(3, k));
        set(symbolBojAx{k},'Matrix', TR);
    end
    drawnow;
end
%% save results
name = 'particle';
robotParticleFilter.landmarks.l = l;
robotParticleFilter.groundTruth.x_true = x_true;
robotParticleFilter.groundTruth.y_true = y_true;
robotParticleFilter.groundTruth.th_true = th_true;
robotParticleFilter.estimation.particles = particles;
robotParticleFilter.estimation.x_hat = x_hat;
robotParticleFilter.estimation.N = N_end;
disp(['saving to dataset/', name, '.mat'])
save(['dataset/', name, '.mat'], 'robotParticleFilter')
%% plot figures
close all
figure
hold on
plot(x_true(1 : N_end), y_true(1 : N_end), 'LineWidth',1)
plot(x_hat(1 : N_end, 1), x_hat(1 : N_end, 2), '-r', 'LineWidth',2)
plot(l(:,1), l(:,2), 'ok', 'LineWidth', 3)
grid on 
axis equal
xlabel('Position X/m')
ylabel('Position Y/m')
legend('ground truth', 'estimated position')
figure
hold on
plot(t(1 : N_end), x_true(1 : N_end), 'LineWidth', 1)
plot(t(1 : N_end), x_hat(1 : N_end, 1), '-r', 'LineWidth',2)
grid on
xlabel('time, s')
ylabel('X position, m')
legend('ground truth', 'estimated position')
figure
hold on
plot(t(1 : N_end), y_true(1 : N_end), 'LineWidth',1)
plot(t(1 : N_end), x_hat(1 : N_end, 2), '-r', 'LineWidth', 2)
grid on
xlabel('time, s')
ylabel('Y position, m')
legend('ground truth', 'estimated position')
figure
hold on
plot(t(1 : N_end), th_true(1 : N_end), 'LineWidth', 1)
plot(t(1 : N_end), x_hat(1 : N_end, 3), '-r', 'LineWidth', 2)
grid on
xlabel('time, s')
ylabel('theta, rad')
legend('ground truth', 'estimated position')
figure
plot(t, N_landmarks)
xlabel('t/s')
ylabel('Number of Landmarks Seen')
figure
hist(N_landmarks, 0:1:12)
xlabel('Number of Landmarks Seen')
ylabel('Counting')
grid on
title('Number of visible landmarks')
%% observation samples
function p_yk_given_xk = GetObservationSample(xk, yk, R, numOfLandmarks, l, LandmarkIndex, d)
    ykSamples = zeros(2 * numOfLandmarks, 1);
    for i = 1 : numOfLandmarks
        ykSamples(2 * i - 1 : 2 *i) = Sys_g(xk, zeros(2,1), l(LandmarkIndex(i),1), l(LandmarkIndex(i),2), d);
    end
    dim1 = max(size(R));
    R_total = zeros(dim1 * numOfLandmarks);
    for i = 1 : numOfLandmarks
        R_total((i - 1) * dim1 + 1 : i * dim1, (i - 1) * dim1 + 1 : i * dim1) = R;
    end
    % measurement error
    Yerror = (yk - ykSamples)';
    % normalize angle error
    for k = 1 : numOfLandmarks
        Yerror(2 * k) = WrapAnlge(Yerror(2 * k));
    end
    p_yk_given_xk = mvnpdf(Yerror, zeros(1, numOfLandmarks * dim1), R_total);
end