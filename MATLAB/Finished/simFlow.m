clc;
clear;
close all;

%% Setup
% Data for Simulation
lidarData = dlmread("UnityData/Corridor/lidar.txt");
kinematicData = dlmread("UnityData/Corridor/velocity.txt");
% lidarData = dlmread("UnityData/ApproachingTower/lidar.txt");
% kinematicData = dlmread("UnityData/ApproachingTower/velocity.txt");

[nu, Map, param] = formatUnityData(lidarData, kinematicData);
param.FOV = pi/2;

cameraAngle = [0, 90, 90];%Rotates into unity co-ordinates

eta = easySphere(cameraAngle, param); 

%Define Parameters
N = length(eta(1, :));
params.N = N;
sigmaR = 50;
target = [0, 0, 1];

u	= zeros(param.resX, param.resY, param.frames);
v	= zeros(param.resX, param.resY, param.frames);
w	= zeros(param.resX, param.resY, param.frames);

flowDiv = zeros(param.resX, param.resY, param.frames);

flowMag = zeros(param.resX, param.resY);
targetCost = zeros(param.resX, param.resY);
movement = zeros(3, param.frames);

% frames collected from unity
frameDir = '.\UnityData\frames';
filePattern = fullfile(frameDir, '*.png');
fileNames = dir(filePattern);
frameArray = cell(1, N);

%calculate optic flow
for kk = 1:param.frames
    for ii = 1:param.resX
        for jj = 1:param.resY
            G = MeasurementModel(nu(:, kk), eta(:, ii, jj), Map(ii, jj, kk));
            u(ii, jj, kk) = G(1);
            v(ii, jj, kk) = G(2);
            w(ii, jj, kk) = G(3);
            targetCost(ii, jj) = dot(eta(:, ii, jj), target);
        end
    end
    % Calculate the divergence of the flow vectors
    flowDiv(:, :, kk) = divergence(u(:, :, kk), v(:, :, kk));
    
    % Normalise the divergence magintudes
    maxDiv = max(max(abs(flowDiv(:, :, kk))));
    if(maxDiv ~= 0)
        flowCost = abs(flowDiv(:, :, kk))./maxDiv;
    else
        flowCost = zeros(param.resX, param.resY);
    end
    
    % Calculate the cost of moving in that direction
    totalCost = flowCost + ((1./targetCost)-1);
    minCost = min(totalCost(:));
    [row,col] = find(totalCost == minCost);
    movement(:, kk) = eta(:, row(1), col(1));
    
    % Assign the target direction to 
%     target = movement(:, kk);
    disp(kk);
end

%% plot the actual data
[a, b, c] = sphere(100);
img = imread('UnityData\Corridor\Corridor.png');

subplot(1, 2, 1);
title("Object Avoidance Algorithm");
hold on;
grid on;
view([30, 20]);
axismin = -0.7;
axismax = 0.7;
xlabel('x');
ylabel('z');
zlabel('y');
daspect([1,1,1]);
axis([axismin, axismax, 0, 1, -.8, axismax]);

u0(:, :) = eta(1, :, :);
v0(:, :) = eta(3, :, :);
w0(:, :) = eta(2, :, :);
divPlot = surf(u0, v0, w0, flowDiv(:, :, 1), 'LineStyle','none', 'FaceAlpha', '0.4');
flowPlot = quiver3(u0 ,v0 ,w0, zeros(30, 30), zeros(30, 30), zeros(30, 30), 2, 'k');

targetDir = quiver3(0, 0, 0, 0, 1, 0, 0,'r'); % Target direction
avoidDir = quiver3(0, 0, 0, 0, 1, 0, 0, 'b');
legend([targetDir, avoidDir], {'Target Direction', 'Avoidance Direction'}, 'Location','northwest');

subplot(1, 2, 2);
img = fliplr(img);
imagePlot = image([axismin, axismax], [axismin, axismax], img);
axis off;
imagePlot.AlphaData = 0.3;
title("Simulated Scene View");
daspect([1,1,1]);

% Annimation
for ii = 1:(param.frames)
    set(avoidDir,  'udata', movement(1, ii),... 
        'vdata', movement(3, ii),...
        'wdata', movement(2, ii) ...
    );

    set(flowPlot,  'udata', u(:, :, ii),... 
        'vdata', v(:, :, ii),...
        'wdata', w(:, :, ii) ...
    );
    set(divPlot, 'cdata', flowDiv(:, :, ii));
    disp(ii);
    pause(0.02);
end

