clc;
clear;
close all;

%% Setup
% Data for Simulation

% !!!!!!!!!! Good Data !!!!!!!!!!!!
% lidarData = dlmread("UnityData/Corridor/Avoiding/lidar.txt");
% kinematicData = dlmread("UnityData/Corridor/Avoiding/velocity.txt");
lidarData = dlmread("UnityData/Corridor/lidar.txt");
kinematicData = dlmread("UnityData/Corridor/velocity.txt");
% lidarData = dlmread("UnityData/ApproachingTower/DirectlyAt/lidar.txt");
% kinematicData = dlmread("UnityData/ApproachingTower/DirectlyAt/velocity.txt");
% !!!!!!!!!! Good Data !!!!!!!!!!!!


[nu, Map, param] = formatUnityData(lidarData, kinematicData);
nuS = nu;

% nu(1, :) = nuS(1, :);
% nu(2, :) = nuS(3, :);
% nu(3, :) = nuS(2, :);
% nu(4, :) = nuS(4, :);
% nu(5, :) = nuS(6, :);
% nu(6, :) = nuS(5, :);

param.fovX = 100 * pi/180;
param.fovY = ((100 * pi / 180) * 9 )/ 16;

param.sigX = 1e-4;
param.sigY = 1e-4;

cameraAngle = [90, 0, 0]; %Rotates into unity co-ordinates

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

% The ratio between these two parameters dictates how tight the vehicle can
% turn

devWeight = 0.5; % Weight of deviationg from target direction
flowWeight = 1; % weight of the divergence

%calculate optic flow
for kk = 1:param.frames
    for ii = 1:param.resX
        for jj = 1:param.resY
            G = simulateSphericalFlow(nu(:, kk), eta(:, ii, jj), Map(ii, jj, kk), param);
%             G = simulateSphericalFlow(nu(:, kk), eta(:, ii, jj), 1, param);
            u(ii, jj, kk) = G(1);
            v(ii, jj, kk) = G(2);
            w(ii, jj, kk) = G(3);
            targetCost(ii, jj) = dot(eta(:, ii, jj), target);
        end
    end
    % Calculate the divergence of the flow vectors
    div = divergence(b, a, v(:, :, kk), u(:, :, kk));
%     div = imgaussfilt(div, 2);
    % Normalise the divergence magintudes
    maxDiv = max(max(abs(div)));
    if(maxDiv ~= 0)
        divCost = abs(div)./maxDiv;
%         divCost = imfilter(divCost, [1, 1;1, 1]);
    else
        divCost = zeros(param.resX, param.resY);
    end
    
    % Calculate the cost of moving in that direction
    totalCost = flowWeight*divCost + devWeight.*(1./targetCost);
    minCost = min(totalCost(:));
    [row,col] = find(totalCost == minCost);
    movement(:, kk) = eta(:, row(1), col(1));
    
    costHist(:, :, kk) = totalCost;
    divHist(:, :, kk) = divCost;
    
    disp(kk);

end


%% plot the actual data

figure;
img = imread('UnityData\Corridor\Corridor.png');
s = 2; %Scale down optic flow vector plotting
% subplot(1, 2, 1);
title("Object Avoidance Algorithm");
hold on;
grid on;
view([10, -70]);
axismin = -0.7;
axismax = 0.7;
xlabel('x');
ylabel('y');
zlabel('z');
daspect([1,1,1]);
axis([axismin, axismax, 0, 1, axismin, axismax]);

u0(:, :) = eta(1, :, :);
v0(:, :) = eta(2, :, :);
w0(:, :) = eta(3, :, :);

divPlot = surf(u0, v0, w0, costHist(:, :, 5), 'LineStyle','none', 'FaceAlpha', '1');
% caxis([1, 1.5]);
flowPlot = quiver3(u0(1:s:end, 1:s:end) ,v0(1:s:end, 1:s:end) ,w0(1:s:end, 1:s:end), zeros(size(u0(1:s:end, 1:s:end))), zeros(size(u0(1:s:end, 1:s:end))), zeros(size(u0(1:s:end, 1:s:end))), 2, 'k');

targetDir = quiver3(0, 0, 0, 0, 1, 0, 0,'r'); % Target direction
avoidDir = quiver3(0, 0, 0, 0, 1, 0, 0, 'b');
legend('Direction Cost', 'Flow Vectors', 'Target Direction', 'Avoidance Direction', 'Location','NorthEast');

% subplot(1, 2, 2);
% img = fliplr(img);
% imagePlot = image([axismin, axismax], [axismin, axismax], img);
% axis off;
% imagePlot.AlphaData = 0.3;
% title("Simulated Scene View");
% daspect([1,1,1]);

% Annimation
for ii = 1:(param.frames)
    set(avoidDir,  'udata', movement(1, ii),... 
        'vdata', movement(2, ii),...
        'wdata', movement(3, ii) ...
    );

    set(flowPlot,  'udata', u(1:s:end, 1:s:end, ii),... 
        'vdata', v(1:s:end, 1:s:end, ii),...
        'wdata', w(1:s:end, 1:s:end, ii) ...
    );
    set(divPlot, 'cdata', costHist(:, :, ii));
    disp(ii);
    pause(0.02);
end
