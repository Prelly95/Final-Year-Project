clc;
clear;
close all;


lidarData = dlmread("UnityData/lidar.txt");
kinematicData = dlmread("UnityData/velocity.txt");

[nu, Map, param] = formatUnityData(lidarData, kinematicData);
eta = easySphere(param.resX, param.resY, pi/2, 0, 90, 90); %Rotates into unity co-ordinates

N = length(eta(1, :));
params.N = N;
sigmaR = 50;

target = [0, 0, 1];

param.frames = length(kinematicData) - 1;

u	= zeros(param.frames, param.resX, param.resY);
v	= zeros(param.frames, param.resX, param.resY);
w	= zeros(param.frames, param.resX, param.resY);


flowMag = zeros(param.resX, param.resY);
targetCost = zeros(param.resX, param.resY);
movement = zeros(3, param.frames);

%calculate optic flow
for ii = 1:param.frames
    for jj = 1:param.resX
        for kk = 1:param.resY
            G = MeasurementModel(nu(:, ii), eta(:, jj, kk), Map(ii, jj, kk));
            u(ii, jj, kk) = G(1);
            v(ii, jj, kk) = G(2);
            w(ii, jj, kk) = G(3);
            flowMag(jj, kk) = norm(G);
            targetCost(jj, kk) = dot(eta(:, jj, kk), target);
        end
    end
    
    maxFlow = max(flowMag(:));
    if(maxFlow ~= 0)
        flowCost = flowMag./maxFlow;
    else
        flowCost = zeros(param.resX, param.resY);
    end
    totalCost = flowCost + ((1./targetCost)-1);
    minCost = min(totalCost(:));
    [row,col] = find(totalCost==minCost);
    movement(:, ii) = eta(:, row(1), col(1));
    target = movement(:, ii);
    disp(ii);
end

%% plot the actual data
[a, b, c] = sphere(100);
imageFiled
for ii = 1:param.frames
    curentImage = imread('UnityData/frames/*.png');
    img{ii} = curentImage;
end

% setting up the plot
figure();

subplot(1, 2, 1);
title("Sim Flow New");
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

surf(a, b, c, 'edgecolor', 'none', 'facecolor', '[0.9, 0.9, 0.9]', 'FaceAlpha', '0.2');
flowPlot = quiver3(eta(1, :, :), eta(2, :, :), eta(3, :, :), zeros(1, 30, 30), zeros(1, 30, 30), zeros(1, 30, 30));
directionPlot = plot3(0, 0, 1, 'rx'); % Target Direction

subplot(1, 2, 2);
imagePlot = image([axismin, axismax], [-axismin, -axismax], img);
imagePlot.AlphaData = 0.3;
daspect([1,1,1]);

%% Annimation
keyboard(); %gives you a chance to move the plot
for ii = 1:(param.frames)
    set(flowPlot,  'udata', u(ii, :, :),... 
                'vdata', v(ii, :, :),...
                'wdata', w(ii, :, :) ...
       );
   set(directionPlot,  'xdata', movement(1, ii),... 
                'ydata', movement(2, ii),...
                'zdata', movement(3, ii) ...
       );
   set(imagePlot, 'CData', img);
    disp(ii);
    pause(0.02);
end



