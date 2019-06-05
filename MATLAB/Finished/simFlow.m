%% Setup

clc;
clear;
close all;

% This data set shows the vehicle following the corridor
% 724 frames - Takes aproximately 
% lidarData = dlmread("UnityData/Corridor/Avoiding/lidar.txt");
% kinematicData = dlmread("UnityData/Corridor/Avoiding/velocity.txt");

% frames - Takes aproximately
% lidarData = dlmread("UnityData/Corridor/lidar.txt");
% kinematicData = dlmread("UnityData/Corridor/velocity.txt");

% frames - ta
% lidarData = dlmread("UnityData/ApproachingTower/DirectlyAt/lidar.txt");
% kinematicData = dlmread("UnityData/ApproachingTower/DirectlyAt/velocity.txt");

% lidarData = dlmread("UnityData/VerticalAvoid/lidar.txt");
% kinematicData = dlmread("UnityData/VerticalAvoid/velocity.txt");

lidarData = dlmread("UnityData/lidar.txt");
kinematicData = dlmread("UnityData/velocity.txt");

disp("Formatting data collected from unity");
[nu, Map, param] = formatUnityData(lidarData, kinematicData);
nuS = nu;

param.fovX = 100 * pi/180;
param.fovY = ((100 * pi / 180) * 9 )/ 16;

param.sigX = 1e-7;
param.sigY = 1e-7;

cameraAngle = [90, 90, 0]; %Rotates into unity co-ordinates

eta = easySphere(cameraAngle, param); 

a(:, :) = eta(1, :, :);
b(:, :) = eta(2, :, :);
c(:, :) = eta(3, :, :);
%Define Parameters
N = length(eta(1, :));
params.N = N;
sigmaR = 50;
target = [0, 0, 1];

u	= zeros(param.resX, param.resY, param.frames);
v	= zeros(param.resX, param.resY, param.frames);
w	= zeros(param.resX, param.resY, param.frames);

yTangentBundle = zeros(param.resX, param.resY, 3);

flowDiv = zeros(param.resX, param.resY, param.frames);

flowMag = zeros(param.resX, param.resY);
targetCost = zeros(param.resX, param.resY);
movement = zeros(3, param.frames);
costHist = zeros(param.resX, param.resY, param.frames);
mag = zeros(param.resX, param.resY);

% frames collected from unity
frameDir = '.\UnityData\frames';
filePattern = fullfile(frameDir, '*.png');
fileNames = dir(filePattern);
frameArray = cell(1, N);

% The ratio between these two parameters dictates how tight the vehicle turns

param.devWeight = 1.3; % Weight of deviationg from target direction
param.flowWeight = 1; % weight of the divergence
param.cutOff = 4; % how many standard deviations of the flow divergence is culled

%calculate optic flow
for kk = 1:param.frames
    for ii = 1:param.resX
        for jj = 1:param.resY
            G = simulateOpticalFlow(nu(:, kk), eta(:, ii, jj), Map(ii, jj, kk), param);
%             G = simulateOpticalFlow([0;0;1;0;0;0], eta(:, ii, jj), 1, param); % for testing 
            u(ii, jj, kk) = G(1);
            v(ii, jj, kk) = G(2);
            w(ii, jj, kk) = G(3);
            
            targetCost(ii, jj) = dot(eta(:, ii, jj), target);
        end
    end
    
    Gf(:, :, 1) = u(:, :, kk);
    Gf(:, :, 2) = v(:, :, kk);
    
    [costHist(:, :, kk), movement(:, kk)] = directionCost(Gf, eta, targetCost, param);
    
    disp(kk);

end


%% plot the actual data
fig = figure;
img = imread('UnityData\Corridor\Corridor.png');
s = 2; %Scale down optic flow vector plotting
% subplot(1, 2, 1);
title("Object Avoidance Algorithm");
hold on;
grid on;
view([15, 10]);
xlabel('C1');
ylabel('C3');
zlabel('C2');
daspect([1,1,1]);
axis([-.8, .8, 0, 1, -.6, .6]);

u0(:, :) = eta(2, :, :);
v0(:, :) = eta(3, :, :);
w0(:, :) = eta(1, :, :);

divPlot = surf(u0, v0, w0, costHist(:, :, 5), 'LineStyle','none', 'FaceAlpha', '1');

targetDir = quiver3(0, 0, 0, 0, 1, 0, 0,'r'); % Target direction
avoidDir = quiver3(0, 0, 0, 0, 1, 0, 0, 'b');
legend('Direction Cost', 'Target Direction', 'Avoidance Direction', 'Location','NorthEast');

% subplot(1, 2, 2);
% img = fliplr(img);
% imagePlot = image([axismin, axismax], [axismin, axismax], img);
% axis off;
% imagePlot.AlphaData = 0.3;
% title("Simulated Scene View");
% daspect([1,1,1]);

% Annimation
for ii = 1:(param.frames)
    set(avoidDir,  'udata', movement(2, ii),... 
        'vdata', movement(3, ii),...
        'wdata', movement(1, ii) ...
    );

    set(divPlot, 'cdata', costHist(:, :, ii));
    disp(ii);
    pause(0.02);
end
