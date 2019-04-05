clc;
clear;
close all;

%% Load Data
res = 30;

eta = easySphere(res, pi/2, 0, -90, 90); %Rotates into unity co-ordinates

Vdata = dlmread("UnityData/velocity.txt");
Ldata = dlmread("UnityData/lidar.txt");
Pdata = dlmread("UnityData/pose.txt");

N = length(eta(1, :));
params.N = N;
sigmaR = 50;

frames = length(Vdata)-5;

Map = zeros(frames, res, res);
% h = heatmap(zeros(30));

for ii = 1:frames
    Map(ii, :, :) = unityReadDist(30, 30, Ldata((ii-1)*30 + 1:(ii-1)*30 + 30, :));
%     a(:,:) = dst(ii, :, :);
%     set(h, 'ColorData', a);
%     pause(0.02);
end

u	= zeros(frames, res, res);
v	= zeros(frames, res, res);
w	= zeros(frames, res, res);

nu(1:3, :) = Vdata(:, 2:4)';
nu(4:6, :) = Vdata(:, 5:7)';

%calculate optic flow
for ii = 1:frames
    for jj = 1:res
        for kk = 1:res
            G = MeasurementModel(nu(:, ii), eta(:, jj, kk), Map(ii, jj, kk));
            u(ii, jj, kk) = G(1);
            v(ii, jj, kk) = G(2);
            w(ii, jj, kk) = G(3);
        end
    end
    disp(ii);
    
end

%% plot the actual data
% setting up the plot
figure();
title("Sim Flow New");
hold on;
grid on;
view([0, 90]);
axismin = -1;
axismax = 1;
xlabel('x');
ylabel('y');
zlabel('z');
axis([axismin, axismax, axismin, axismax, axismin, axismax]);

[a, b, c] = sphere(100);
% surf(a, b, c, 'edgecolor', 'none', 'facecolor', '[0.8, 0.8, 0.8]', 'FaceAlpha', '0.8');

plot1 = quiver3(eta(1, :, :), eta(2, :, :), eta(3, :, :), zeros(1, 30, 30), zeros(1, 30, 30), zeros(1, 30, 30));
plot2 = quiver3(0, 0, 0, nu(1, 1), nu(2, 1), nu(3, 1));

%%Annimation
for ii = 1:(frames-10)
    set(plot1,  'udata', u(ii, :, :),... 
                'vdata', v(ii, :, :),...
                'wdata', w(ii, :, :) ...
       );
    disp(ii);
    pause(0.02);
end



