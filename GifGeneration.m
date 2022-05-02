%% plot results 
clear
close all
load('dataset/dataset2.mat')
load('dataset/AMCL.mat')
%% define whether to generate git
frame_start = 1200;
frame_size = 512;
frame_end = frame_start + frame_size - 1;
filename_gif = 'gifoutput/acm_output.gif';
record_gif = true;
%% plot figures
figure
hold on
plot(robot.system.groundTruth.x(frame_start : frame_end ), robot.system.groundTruth.y(frame_start : frame_end ), 'LineWidth',1)
plot(filter.estimation.X_hat(frame_start : frame_end , 1), filter.estimation.X_hat(frame_start : frame_end , 2), '-r', 'LineWidth',2)
plot(robot.system.sensors.landmarks(:, 1), robot.system.sensors.landmarks(:, 2), 'ok', 'LineWidth', 3)
grid on 
axis equal
xlabel('Position X/m')
ylabel('Position Y/m')
legend('ground truth', 'estimation', 'landmarks')
title('estimation results (x and y plane)')
figure
hold on
plot(robot.system.time (frame_start : frame_end ), robot.system.groundTruth.x(frame_start : frame_end ), 'LineWidth', 1)
plot(robot.system.time (frame_start : frame_end ), filter.estimation.X_hat(frame_start : frame_end , 1), '-r', 'LineWidth',2)
grid on
xlabel('time, s')
ylabel('X position, m')
legend('ground truth', 'estimation')
title('estimation results (x direction)')
figure
hold on
plot(robot.system.time (frame_start : frame_end ), robot.system.groundTruth.y(frame_start : frame_end ), 'LineWidth',1)
plot(robot.system.time (frame_start : frame_end ), filter.estimation.X_hat(frame_start : frame_end, 2), '-r', 'LineWidth', 2)
grid on
xlabel('time, s')
ylabel('Y position, m')
legend('ground truth', 'estimation')
title('estimation results (y direction)')
figure
hold on
plot(robot.system.time(frame_start : frame_end ), robot.system.groundTruth.theta(frame_start : frame_end ), 'LineWidth', 1)
plot(robot.system.time(frame_start : frame_end ), filter.estimation.X_hat(frame_start : frame_end , 3), '-r', 'LineWidth', 2)
grid on
xlabel('time, s')
ylabel('theta, rad')
legend('ground truth', 'estimation')
title('estimation results (theta)')
figure
subplot(2, 1, 1)
plot(robot.system.time(frame_start : frame_end ), filter.estimation.numberOfLandmarks(frame_start : frame_end ), 'LineWidth', 1)
grid on
xlabel('time, s')
ylabel('landmark number')
title('number of visible landmarks')
subplot(2, 1, 2)
semilogy(robot.system.time(frame_start : frame_end ), filter.estimation.numberOfRequiredParticles(frame_start : frame_end ), 'LineWidth', 1)
grid on
xlabel('time, s')
ylabel('number of required particles')
title('number of required particles')
%% define video settings
if record_gif
    anim1 = figure('Units','normalized','OuterPosition',[0.1 0.1 0.4 0.7]);
    ax = axes('Position',[0.08 0.1 0.90 0.90],'XLim',[-3 12],'YLim',[-5 6],...
        'ZLim',[-1 1],'DataAspectRatio', [1 1 1]); % axis for the 
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
    % axis equal
    xlabel('Position X/m')
    ylabel('Position Y/m')
    %% attach the symbol to rotation
    TR = GetTransformMatrix(x_true(1), y_true(1), th_true(1));
    set(symbolBojAx{filter.settings.NsMax + 1},'Matrix',TR);
    for k = 1 : filter.settings.Ncurrent
        TR = GetTransformMatrix(filter.estimation.particles{1}(1, k),...
                                filter.estimation.particles{1}(2, k),...
                                filter.estimation.particles{1}(3, k));
        set(symbolBojAx{k},'Matrix',TR);
        set(symbolBoj{k},'visible', 'on');
    end
    %% draw the frames
    for i = frame_start : frame_end 
        %% determine how many land marks are seem
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
        %% load image to gif
        frame = getframe(anim1);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if i == frame_start
            imwrite(imind,cm,filename_gif,'gif', 'Loopcount',inf, 'DelayTime', robot.system.sensors.samplePeriod );
        else
            imwrite(imind,cm,filename_gif,'gif','WriteMode','append', 'DelayTime', robot.system.sensors.samplePeriod);
        end        
    end
end
