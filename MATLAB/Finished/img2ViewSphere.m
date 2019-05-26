clc;
clear;
close all;

sR = 10; % Scale Ratio, speeds up plotting

cropX = 1;
cropY = 1;

VideoFile = 'GoProData/Static_Scene.MP4';
v = VideoReader(VideoFile);
frame = readFrame(v);

distFile = 'GoProData/distCoeffs.json';
distJson = jsondecode(fileread(distFile));
distVector6 = distJson.M';

camFile = 'GoProData/camMatrix.json';
camJson = jsondecode(fileread(camFile));
camTemp = camJson.M;
camMatrix = reshape(camTemp, 3, 3);

param.camResX = v.Width;
param.camResY = v.Height;

param.camMatrix = camMatrix;
param.distVec = distVector6;

param.fovX = 2.0630;
param.fovY = 1.2130;

[X, Y] = meshgrid(cropX:sR:v.Width-cropX, cropY:sR:v.Height-cropY);

% Gathering colour data for undistorted points
r(:, :) = double(frame(cropX:sR:end-cropX, cropY:sR:end-cropY, 1))/255;
g(:, :) = double(frame(cropX:sR:end-cropX, cropY:sR:end-cropY, 2))/255;
b(:, :) = double(frame(cropX:sR:end-cropX, cropY:sR:end-cropY, 3))/255;

cd(:, :, :) = [r(:), g(:), b(:)];

unDistPoints = pix2vec([X(:), Y(:)], param, 1);
camPoints = pix2vec([X(:), Y(:)], param, 0);

fig = figure;
sp1 = subplot(1, 2, 1);
scatter3(sp1, unDistPoints(:, 1), unDistPoints(:, 2), unDistPoints(:, 3), [], cd, 'filled', 'SizeData', 2);
view([-20, 20]);
axis([-1, 1, -1, 1, -1, 1]);
title("Image Undistorted and projected onto the View Sphere");

sp2 = subplot(1, 2, 2);
scatter3(sp2, camPoints(:, 1), camPoints(:, 2), camPoints(:, 3), [], cd, 'filled', 'SizeData', 2);
view([-20, 20]);
axis([-1, 1, -1, 1, -1, 1]);
title("Original Image projected onto the View Sphere");
