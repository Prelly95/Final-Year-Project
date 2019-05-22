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
eta = easySphere(param.resX, param.resY, pi/2, 0, 90, 90); %Rotates into unity co-ordinates
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

% for ii = 1:param.frames
%     baseFileName = fileNames(ii).name;
%     fullFileName = fullfile(frameDir, baseFileName);
%     frameArray{ii} = imread(fullFileName);
% end

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
img = imread('UnityData/frames/000001.png');
% for ii = 1:param.frames
%     curentImage = imread('UnityData/frames/*.png');
%     img{ii} = curentImage;
% end

% setting up the plot
% figure();

subplot(1, 2, 1);
title("Predicted Optical Flow");
hold on;
grid on;
view([180, -90]);
axismin = -1;
axismax = 1;
xlabel('x');
ylabel('y');
zlabel('z');
daspect([1,1,1]);
axis([axismin, axismax, axismin, axismax, axismin, axismax]);

% surf(a, b, c, 'edgecolor', 'none', 'facecolor', '[0.9, 0.9, 0.9]', 'FaceAlpha', '0.2');
u0(:, :) = eta(1, :, :);
v0(:, :) = eta(2, :, :);
w0(:, :) = eta(3, :, :);
divPlot = surf(u0, v0, w0, flowDiv(:, :, 1), 'LineStyle','none', 'FaceAlpha', '0.4');
flowPlot = quiver3(u0 ,v0 ,w0, zeros(30, 30), zeros(30, 30), zeros(30, 30), '.');
directionPlot = plot3(0, 0, 1, 'rx', 'LineWidth', 2); % Direction moving
plot3(0, 0, 1, 'bd', 'LineWidth', 2); % Target direction

subplot(1, 2, 2);
imagePlot = image([axismin, axismax], [axismin, axismax], img);
imagePlot.AlphaData = 0.3;
daspect([1,1,1]);

% Annimation
% keyboard(); %gives you a chance to move the plot
for ii = 1:(param.frames)
    set(directionPlot,  'xdata', movement(1, ii),... 
        'ydata', movement(2, ii),...
        'zdata', movement(3, ii) ...
    );
    set(flowPlot,  'udata', u(:, :, ii),... 
        'vdata', v(:, :, ii),...
        'wdata', w(:, :, ii) ...
    );
   set(divPlot, 'cdata', flowDiv(:, :, ii));
%    set(imagePlot, 'CData', img);
    disp(ii);
    pause(0.02);
end

