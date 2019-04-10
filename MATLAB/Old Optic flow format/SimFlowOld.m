clc;
clear;
close all;

%% Load Data
res = 30;
[x, y, z] = easySphereOld(res, pi/2, 0, 0, 90);
Vdata = dlmread("UnityData/velocity.txt");
Ldata = dlmread("UnityData/lidar.txt");

N = length(x);
params.N = N;
sigmaR = 50;

frames = length(Vdata)-5;

dst = zeros(frames, res, res);
% h = heatmap(zeros(30));

for ii = 1:frames
    dst(ii, :, :) = unityReadDist(30, 30, Ldata((ii-1)*30 + 1:(ii-1)*30 + 30, :));
%     a(:,:) = dst(ii, :, :);
%     set(h, 'ColorData', a);
%     pause(0.02);
end

u	= zeros(frames, N);
v	= zeros(frames, N);
w	= zeros(frames, N);

rPC		= zeros(3, N);
rQC(1, :) = x;
rQC(2, :) = y;
rQC(3, :) = z;

vCN(:, 1) = Vdata(:, 1); % Translational velocity of the camera in world coordinates
vCN(:, 2) = Vdata(:, 2);
vCN(:, 3) = Vdata(:, 3);
omegaBN = Vdata(:, 5:7); % Angular velocity of the camera in the world coordinates 

%calculate optic flow
for kk = 1:frames
    
    rPC(1, :) = x.*(dst(kk, :));
    rPC(2, :) = y.*(dst(kk, :));
    rPC(3, :) = z.*(dst(kk, :));

    for jj = 1:N
%         Look at calculating rPC in here to take into account the pose of
%         the vehicle
        [u(kk, jj),  v(kk, jj), w(kk, jj)] = MeasurementModelOld(rQC(:, jj), rPC(:, jj), vCN(kk, :)', omegaBN(kk, :)');
    end
    disp(kk);
end

%% plot the actual data

% setting up the plot
figure();
title("Sim Flow old");
hold on;
grid on;
view([0, 0]);
axismin = -1;
axismax = 1;
axis([axismin, axismax, axismin, axismax, axismin, axismax]);

[a, b, c] = sphere(100);
% surf(a, b, c, 'edgecolor', 'none', 'facecolor', '[0.8, 0.8, 0.8]', 'FaceAlpha', '0.8');

plot1 = quiver3(x, y, z, zeros(1, N), zeros(1, N), zeros(1, N));
%%
for ii = 1:(frames-10)
    set(plot1,  'udata', u(ii, :),... 
                'vdata', v(ii, :),...
                'wdata', w(ii, :) ...
       );
    disp(ii);
    pause(0.02);
end



