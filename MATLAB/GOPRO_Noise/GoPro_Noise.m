clc;
clear;
close all;

% VideoFile = 'Calibration_Vid.MP4';
% VideoFile = 'Static_Scene.MP4';
VideoFile = 'GX010113.MP4';
% load('CameraParameters_GoPro.mat');

distFile = 'distCoeffs.json';
distJson = jsondecode(fileread(distFile));
distVector = distJson.M';

camFile = 'camMatrix.json';
camJson = jsondecode(fileread(camFile));
camTemp = camJson.M;
camMatrix = reshape(camTemp, 3, 3);

v = VideoReader(VideoFile);

param.camResX = v.Width;
param.camResY = v.Height;

param.camMatrix = camMatrix;
param.distVec = distVector;

param.fovX = 1.2130;
param.fovY = 2.0630;


N = 130;

sR = 10; %Sample Ratio

% Optical flow - setup
flowMethod = opticalFlowFarneback;
% flowMethod = opticalFlowHS;
% flowMethod = opticalFlowLK;
% flowMethod = opticalFlowLKDoG;

Gf = zeros(v.Height/sR, v.Width/sR, 2, N);

sampleX = zeros(N, 1);
sampleY = zeros(N, 1);
%%
for ii = 1:N
    vidFrame = readFrame(v);

    G_Cell = estimateFlow(flowMethod,rgb2gray(vidFrame));
    Gf(:, :, 1, ii) = G_Cell.Vx(1:sR:end, 1:sR:end);
    Gf(:, :, 2, ii) = G_Cell.Vy(1:sR:end, 1:sR:end);

    sampleX(ii) = Gf(106, 240, 1, ii);
    sampleY(ii) = Gf(106, 240, 2, ii);
    disp(ii);
end

%%
close all;
a = rmoutliers(sampleX);
b = rmoutliers(sampleY);
% a = rmoutliers(sampleX(1:10));
% b = rmoutliers(sampleY(1:10));

x = linspace(-5*std(a), 5*std(a), 1000)*10^0;
y = linspace(-5*std(b), 5*std(b), 1000)*10^0;

figure;
subplot(1, 2, 1);
histogram(a, 15, 'Normalization', 'pdf');
hold on;
plot(x, normpdf(x, mean(a), std(a)));
xlabel('Flow Magnitude');
ylabel('Probability Density');
title('GoPro Noise PDF Gx');

% figure;
% hold on;
subplot(1, 2, 2);
histogram(b, 15, 'Normalization', 'pdf');
hold on;
plot(y, normpdf(y, mean(b), std(b)));
xlabel('Flow Magnitude');
ylabel('Probability Density');
title('GoPro Noise PDF Gy');

%%
figure;
[X, Y] = meshgrid(1:sR:v.Width, 1:sR:v.Height);

x = X(:);
y = Y(:);

[c, d] = undistortPoint_mex(x, y, camMatrix, distVector);

tempX(:, :) = Gf(:, :, 1, 1);
tempY(:, :) = Gf(:, :, 2, 1);

uX = tempX(:);
uY = tempY(:);

h = quiver(c, -d, uX, uY);

axis([-1400, 5400, -2800, 800]);
for ii = 2:N
    
    tempX(:, :) = Gf(:, :, 1, ii);
    tempY(:, :) = Gf(:, :, 2, ii);

    uX = tempX(:);
    uY = tempY(:);

    set(h, 'udata', uX, 'vdata', uY);
    pause(.02);
end

