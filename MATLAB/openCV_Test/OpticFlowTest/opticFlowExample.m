clc;
clear;
close all;

% Reading image - setup
frameDir = '.\frames';
filePattern = fullfile(frameDir, '*.png');
fileNames = dir(filePattern);
testImg =  imread('.\frames\000001.png');

N = length(fileNames);
[x, y, ~] = size(testImg);

% Optical flow - setup
farnback = opticalFlowFarneback;
initFlow = estimateFlow(farnback,rgb2gray(testImg));
frameArray = cell(1, N);
Gf = cell(1, N);

div = zeros(x, y, N);
flow = zeros(x, y, 2, N);

%% Calculate the flow
for ii = 1:N
    baseFileName = fileNames(ii).name;
    fullFileName = fullfile(frameDir, baseFileName);
    frameArray{ii} = imread(fullFileName);
    currentFrame(:, :, :) = frameArray{ii};
    grayFrame = rgb2gray(currentFrame);
    
    Gf = estimateFlow(farnback,grayFrame);
    flow(:, :, 1, ii) = Gf.Vx;
    flow(:, :, 2, ii) = Gf.Vy;
    div(:, :, ii) = divergence(flow(:, :, 1, ii), flow(:, :, 2, ii));
    disp(ii);
end

%% Plot the data

% Plotting - setup
% h = figure;
% GfPlot = axes(h,'Position',[0 0 1 1],'Title','Plot of Optical Flow Vectors');
% keyboard;
% for ii = 1:length(fileNames)
%     currentImage = frameArray{ii};
%     imshow(frameArray{ii});
%     hold on;
%     plot(Gf{ii},'DecimationFactor',[5 5],'ScaleFactor',2,'Parent',GfPlot);
%     hold off;
%     pause(0.02);
%     disp(ii);
% end
close;

offset = min(min(min(div)));
if(offset > 0)
    offset = 0;
else
    offset = -offset + eps;
end

% keyboard;
div = div + offset;

figure;
h = surf(div(:, :, 1), 'LineStyle','none');
lightangle(45, 90);
h.AmbientStrength = .5;
h.DiffuseStrength = .5;
h.SpecularStrength = .1;
h.SpecularExponent = 25;
h.BackFaceLighting = 'unlit';

for ii = 1:length(fileNames)
    set(h,  'zdata', div(:, :, ii), ...
            'cdata', h.CData);
    view(0, -90);
    pause(0.02);
    disp(ii);

end