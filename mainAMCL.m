%% using the particle filter
clear
close all
load('dataset/dataset2.mat')
%% set robot parameters
robot.system.numOfStates = 3;
robot.system.control.input = [v om];% assemble input.
robot.system.control.Q = diag([v_var om_var]);
robot.system.time = t;
robot.system.sensors.maxRange = 2;
robot.system.sensors.R = diag([r_var b_var]);
robot.system.sensors.samplePeriod = 0.1; % sample time 0.1s
robot.system.sensors.NumberOfMeasurements = max(size(x_true));
robot.system.sensors.range = r;
robot.system.sensors.bearing = b;
robot.system.sensors.landmarks = l;
robot.system.sensors.offset = d;

robot.system.groundTruth.x = x_true;
robot.system.groundTruth.y = y_true;
robot.system.groundTruth.theta = th_true;

robot.system.sensors.visibleLandMarks = ...
    zeros(17, robot.system.sensors.NumberOfMeasurements);

[robto.sampleSize, ~] = size(x_true);
%% initialize amcl
filter.settings.NsMax = 500;
filter.settings.NsMin = 20;
filter.settings.Ninitial = filter.settings.NsMax;
filter.settings.Ncurrent = filter.settings.Ninitial;
filter.seetings.upperQuantile = 3;
filter.settings.eposilon = 0.15;
filter.seetings.binResolution.x = 0.2;
filter.seetings.binResolution.y = 0.2;
filter.seetings.binResolution.theta = 5 / 56.3;

filter.estimation.particles = cell(robto.sampleSize, 1);
filter.estimation.particles{1} = zeros(robot.system.numOfStates, filter.settings.Ninitial );
filter.estimation.weights = cell(robto.sampleSize, 1);
filter.estimation.weights{1} = ones(filter.settings.Ninitial, 1) / filter.settings.Ninitial; % all particles have the same weight
for i = 1 : filter.settings.Ninitial  
    % sample initial states
    filter.estimation.particles{1}(1, i) = -4 + 16 * rand(1, 1);     % at time k=1
    filter.estimation.particles{1}(2, i) = -5 + 10 * rand(1, 1);
    filter.estimation.particles{1}(3, i) = WrapAnlge(-pi + 2 * pi * rand(1, 1));
end  
filter.estimation.numberOfLandmarks = zeros(robto.sampleSize, 1);
filter.estimation.X_hat = zeros(robto.sampleSize, 3);
filter.estimation.numberOfRequiredParticles = zeros(robto.sampleSize, 1);
filter.estimation.numberOfRequiredParticles(1) = filter.settings.Ninitial;
%% define video settings
anim1 = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.7]);
ax = axes('Position',[0.08 0.1 0.90 0.90],'XLim',[-3 12],'YLim',[-5 6],...
    'ZLim',[-1 1],'DataAspectRatioMode','manual'); % axis for the 
t1 = title({['number of particles: ', num2str(filter.settings.Ninitial)], ['number of landmarks: ' num2str(0)]});
anim1.Renderer='Painters';
% define triangle symbol
l_symbol = 0.2;
h_symbol = 0.4;
faceCoordinates = [h_symbol * 2 /3, 0, 0;
                  -h_symbol * 1 /3, l_symbol / 2, 0;
                  -h_symbol * 1 /3, -l_symbol / 2, 0];
vertices_index = [1, 2, 3];
% generate symbols based on maximum particle number
% active symbols are used based on the number of current particles
for i = 1 : filter.settings.NsMax
    % patch corresponding to particles
    symbolBojAx{i} = hgtransform('Parent',ax);
    symbolBoj{i} = patch('Faces', vertices_index, 'Vertices', faceCoordinates, 'FaceColor','cyan');
    set(symbolBoj{i},'Parent', symbolBojAx{i});
    set(symbolBoj{i},'visible', 'off');
end
% the last symbol is reserved for the ground truth
symbolBojAx{filter.settings.NsMax + 1} = hgtransform('Parent',ax);
symbolBoj{filter.settings.NsMax + 1} = patch('Faces', vertices_index, 'Vertices', faceCoordinates, 'FaceColor','blue');
set(symbolBoj{filter.settings.NsMax + 1},'Parent', symbolBojAx{filter.settings.NsMax + 1});
% define the landmarks
N_gamma = 20;
gamma = linspace(-pi, pi, N_gamma);
radius = 0.1;
[di1, di2] = size(robot.system.sensors.landmarks);
landmarkCoordinates = zeros(di1, 3);
for i = 1 : N_gamma
    landmarkCoordinates(i, :) = radius * [cos(gamma(i)), sin(gamma(i)), 0];
end
for i = 1 : di1
    offSet = [ones(N_gamma, 1) * robot.system.sensors.landmarks(i, 1) ones(N_gamma, 1) * robot.system.sensors.landmarks(i, 2) zeros(N_gamma, 1)];
    landmarkSymbol{i} = patch('Faces', 1:1:20 , 'Vertices', landmarkCoordinates + offSet, 'FaceColor','black');
    lidarIndicator{i} = line('XData', [robot.system.sensors.landmarks(i, 1) robot.system.groundTruth.x(i)],...
                             'YData', [robot.system.sensors.landmarks(i, 2) robot.system.groundTruth.y(i)],...
                             'LineWidth', 1, 'Color', 'red');
    set(lidarIndicator{i}, 'visible', 'off');
end
grid on 
axis equal
xlabel('Position X/m')
ylabel('Position Y/m')
%% attach the symbol to histogram
TR = GetTransformMatrix(x_true(1), y_true(1), th_true(1));
set(symbolBojAx{filter.settings.NsMax + 1},'Matrix',TR);
for k = 1 : filter.settings.Ncurrent
    TR = GetTransformMatrix(filter.estimation.particles{1}(1, k),...
                            filter.estimation.particles{1}(2, k),...
                            filter.estimation.particles{1}(3, k));
    set(symbolBojAx{k},'Matrix',TR);
    set(symbolBoj{k},'visible', 'on');
end
drawnow;
%% run the estimator
N_end = 3300;
robot.N_end = N_end;
for i = 2 : N_end
    %% determine how many land marks are seem
    for j = 1 : 17
        if robot.system.sensors.range(i, j) == 0
        elseif robot.system.sensors.range(i, j) > robot.system.sensors.maxRange
        else
            filter.estimation.numberOfLandmarks(i) = filter.estimation.numberOfLandmarks(i) + 1;
            robot.system.sensors.visibleLandMarks(filter.estimation.numberOfLandmarks(i), i) = j; % assign corresponding landmarks index to array
        end
    end    
    if filter.estimation.numberOfLandmarks(i) ~= 0
        % if there are visible landmarks, run the AMCL
        % need to dynamically generate samples and bins
        new_particles = [];
        bins_with_support = [];
        number_of_new_particles = 0;
        number_of_bins_with_support = 0;
        number_of_required_particles = filter.settings.NsMin;
        % firstly, calculate the cumulative sum of all the weights
        cumsumWk = cumsum(filter.estimation.weights{i - 1});
        % compute the measurement at this time step
        yk = zeros(2 * filter.estimation.numberOfLandmarks(i), 1);
        for k = 1 : filter.estimation.numberOfLandmarks(i) 
            % assemble observation matrix by number of landmarks seen
            yk(2*k-1) = robot.system.sensors.range(i, robot.system.sensors.visibleLandMarks(k, i));
            yk(2*k)   = robot.system.sensors.bearing(i, robot.system.sensors.visibleLandMarks(k, i));
        end
        filter.estimation.weights{i} = [];
        filter.estimation.particles{i} = [];
        while number_of_new_particles < filter.settings.Ncurrent
            number_of_new_particles = number_of_new_particles + 1;
            % first, need to draw a sample based on the particle weights
            % index_j = generate_sample_index(self.particles)
            index_j = SampleIndex(filter.estimation.weights{i - 1},  cumsumWk , 'naive search');
            % Propagate state of selected particle
            % propaged_state = self.propagate_sample(self.particles[index_j][1],
            %                                        robot_forward_motion,
            %                                        robot_angular_motion)
            filter.estimation.particles{i}(:, number_of_new_particles)...
                = Sys_f(filter.estimation.particles{i - 1}(:, index_j),...
                                    robot.system.control.input(i, :)',...
                                    normrnd(zeros(2, 1), 50 * [v_var om_var]'),...
                                    robot.system.sensors.samplePeriod );
            % wrap the angle
            filter.estimation.particles{i}(3, number_of_new_particles) ...
                = WrapAnlge(filter.estimation.particles{i}(3, number_of_new_particles));   
            % Compute the weight that this propagated state would get with the current measurement
            % importance_weight = self.compute_likelihood(propaged_state, measurements, landmarks)
            filter.estimation.weights{i}(number_of_new_particles)...
                = GetObservationSample(filter.estimation.particles{i}(:, number_of_new_particles),...
                                                     yk, 15* robot.system.sensors.R,...
                                                     filter.estimation.numberOfLandmarks(i),...
                                                     robot.system.sensors.landmarks,...
                                                     robot.system.sensors.visibleLandMarks(:, i), robot.system.sensors.offset); 
            % Next, we convert the discrete distribution of all new samples into a histogram. We must check if the new
            % state (propagated_state) falls in a histogram bin with support or in an empty bin. We keep track of the
            % number of bins with support. Instead of adopting a (more efficient) tree, a simple list is used to
            % store all bin indices with support since there is are no performance requirements for our use case.
            % Map state to bin indices
            indces = [floor(filter.estimation.particles{i}(1, number_of_new_particles) / filter.seetings.binResolution.x),...
                      floor(filter.estimation.particles{i}(2, number_of_new_particles) / filter.seetings.binResolution.y),...
                      floor(filter.estimation.particles{i}(3, number_of_new_particles) / filter.seetings.binResolution.theta)];
            %  Add indices if this bin is empty (i.e. is not in list yet)
            %  if indices not in bins_with_support:
            %  bins_with_support.append(indices)
            %  number_of_bins_with_support += 1
            if isempty(bins_with_support) || ~ismember(indces, bins_with_support, 'rows')
                number_of_bins_with_support  = number_of_bins_with_support + 1;
                bins_with_support(number_of_bins_with_support, :) = indces;
            end
            %  Update number of required particles (only defined if number of bins with support above 1)
            %  if number_of_bins_with_support > 1:
            %  number_of_required_particles = compute_required_number_of_particles_kld(number_of_bins_with_support,
            %                                                                           self.epsilon,
            %                                                                       self.upper_quantile)
            if number_of_bins_with_support > 1
                filter.settings.Ncurrent = GetNumOfRequiredSamples(filter.settings.NsMax,...
                                                                   number_of_bins_with_support,...
                                                                   filter.settings.eposilon,...
                                                                   filter.seetings.upperQuantile);
            end
        end
        % normalize weights
        filter.estimation.weights{i} = filter.estimation.weights{i} / norm(filter.estimation.weights{i});
    else
        % otherwise, just direct propagate particles and keep the number
        % of paticles the same
        filter.estimation.weights{i} = filter.estimation.weights{i - 1};
        filter.estimation.particles{i} = zeros(size(filter.estimation.particles{i - 1}));
        for k = 1 : length(filter.estimation.weights{i})
            filter.estimation.particles{i}(:, k)...
            = Sys_f(filter.estimation.particles{i - 1}(:, k),...
                    robot.system.control.input(i, :)',...
                    normrnd(zeros(2, 1), 10 * [v_var om_var]'),...
                    robot.system.sensors.samplePeriod );
        end
    end
    filter.estimation.numberOfRequiredParticles(i) = length(filter.estimation.weights{i});
    %% Compute estimated state
    % pick the particle with the largest weight as the estimation
    [~, Iest] = max(filter.estimation.weights{i});
    filter.estimation.X_hat(i, :) = filter.estimation.particles{i}(:, Iest)';
    %% show the points
    % change the particle position and ground truth
    TR = GetTransformMatrix(robot.system.groundTruth.x(i),...
                            robot.system.groundTruth.y(i),....
                            robot.system.groundTruth.theta(i));
    set(symbolBojAx{filter.settings.NsMax + 1},'Matrix',TR);
    len = length(filter.estimation.weights{i});
    for k = 1 : len
        TR = GetTransformMatrix(filter.estimation.particles{i}(1, k),...
                                filter.estimation.particles{i}(2, k),...
                                filter.estimation.particles{i}(3, k));
        set(symbolBojAx{k},'Matrix', TR);
        set(symbolBoj{k},'visible', 'on');
    end
    % disable unused particles
    for k = len + 1 : filter.settings.NsMax
        set(symbolBoj{k},'visible', 'off');
    end
    t1.String{1} = ['number of particles: ', num2str(filter.settings.Ncurrent)];
    t1.String{2} = ['number of landmarks: ' num2str(filter.estimation.numberOfLandmarks(i))];
    for k = 1 : filter.estimation.numberOfLandmarks(i)        
        set(lidarIndicator{k},...
            'XData', [robot.system.sensors.landmarks(robot.system.sensors.visibleLandMarks(k, i), 1) robot.system.groundTruth.x(i)],...
            'YData', [robot.system.sensors.landmarks(robot.system.sensors.visibleLandMarks(k, i), 2) robot.system.groundTruth.y(i)],...
            'visible', 'on');
    end
    for k = filter.estimation.numberOfLandmarks(i) + 1 : di1
        set(lidarIndicator{k}, 'visible', 'off');
    end
    drawnow;
end
%% save results
name = 'AMCL';
disp(['saving to ', name, '.mat'])
save(['dataset/', name, '.mat'], 'robot', 'filter')
%% plot figures
figure
hold on
plot(robot.system.groundTruth.x(1 : N_end), robot.system.groundTruth.y(1 : N_end), 'LineWidth',1)
plot(filter.estimation.X_hat(1 : N_end, 1), filter.estimation.X_hat(1 : N_end, 2), '-r', 'LineWidth',2)
plot(robot.system.sensors.landmarks(:,1), robot.system.sensors.landmarks(:,2), 'ok', 'LineWidth', 3)
grid on 
axis equal
xlabel('Position X/m')
ylabel('Position Y/m')
legend('ground truth', 'estimation', 'landmarks')
title('estimation results (x and y plane)')
figure
hold on
plot(robot.system.time (1 : N_end), robot.system.groundTruth.x(1 : N_end), 'LineWidth', 1)
plot(robot.system.time (1 : N_end), filter.estimation.X_hat(1 : N_end, 1), '-r', 'LineWidth',2)
grid on
xlabel('time, s')
ylabel('X position, m')
legend('ground truth', 'estimation')
title('estimation results (x direction)')
figure
hold on
plot(robot.system.time (1 : N_end), robot.system.groundTruth.y(1 : N_end), 'LineWidth',1)
plot(robot.system.time (1 : N_end), filter.estimation.X_hat(1 : N_end, 2), '-r', 'LineWidth', 2)
grid on
xlabel('time, s')
ylabel('Y position, m')
legend('ground truth', 'estimation')
title('estimation results (y direction)')
figure
hold on
plot(robot.system.time(1 : N_end), robot.system.groundTruth.theta(1 : N_end), 'LineWidth', 1)
plot(robot.system.time(1 : N_end), filter.estimation.X_hat(1 : N_end, 3), '-r', 'LineWidth', 2)
grid on
xlabel('time, s')
ylabel('theta, rad')
legend('ground truth', 'estimation')
title('estimation results (theta)')
figure
subplot(2, 1, 1)
plot(robot.system.time(1 : N_end), filter.estimation.numberOfLandmarks(1 : N_end), 'LineWidth', 1)
grid on
xlabel('time, s')
ylabel('landmark number')
title('number of visible landmarks')
subplot(2, 1, 2)
semilogy(robot.system.time(1 : N_end), filter.estimation.numberOfRequiredParticles(1 : N_end), 'LineWidth', 1)
grid on
xlabel('time, s')
ylabel('number of required particles')
title('number of required particles')
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
%% get required number of samples
function n = GetNumOfRequiredSamples(maxSampleSize, k, eposilon, upperquantile)
      % Compute the number of samples needed within a particle filter when k bins in the multidimensional histogram contain
      % samples. Use Wilson-Hilferty transformation to approximate the quantiles of the chi-squared distribution as proposed
      % by Fox (2003).
      temp = 1 - 2 / (9 * (k - 1)) + sqrt(2 / (9 * (k - 1))) * upperquantile;
      n = ceil((k - 1) / (2 * eposilon) * temp ^3);
      if n > maxSampleSize
          n = maxSampleSize;
      end
end
%% 1- delta quantile of standard normal distribution
function z = GetQuantileNormal(delta)
    
end