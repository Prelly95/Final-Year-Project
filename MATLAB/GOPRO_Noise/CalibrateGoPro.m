clc;
clear;
close all;

sR = 10; % scale Ratio

VideoFile = 'Static_Scene.MP4';
v = VideoReader(VideoFile);
frame = readFrame(v);

% C# 6th order Undistortion
distFile = 'distCoeffs.json';
distJson = jsondecode(fileread(distFile));
distVector6 = distJson.M';

camFile = 'camMatrix.json';
camJson = jsondecode(fileread(camFile));
camTemp = camJson.M;
camMatrix = reshape(camTemp, 3, 3);

%%
if(~exist('cameraParams', 'var'))
    try
        load('CameraParameters_GoPro.mat');
    catch
        disp("Couldnt find CameraParameters_GoPro.mat in this folder");
        disp("Calculating new CameraParams - may take some time");
    
        images = imageDatastore('C:\Users\Patrick\Documents\Uni Work\FYP Part B\Final-Year-Project\MATLAB\GOPRO_Noise\Calibration_frames');
        [imagePoints,boardSize] = detectCheckerboardPoints(images.Files);

        squareSize = 29;
        worldPoints = generateCheckerboardPoints(boardSize,squareSize);

        I = readimage(images,1);
        imageSize = [size(I,1),size(I,2)];
        cameraParams = estimateCameraParameters(imagePoints,worldPoints, 'ImageSize',imageSize, 'NumRadialDistortionCoefficients', 3, 'EstimateTangentialDistortion', true);
        
        save('CameraParameters_GoPro.mat', 'cameraParams');
    end
end
% paramStruct  = cameraParams.toStruct;

%%
rDist = cameraParams.RadialDistortion;
tDist = cameraParams.TangentialDistortion;

distVector3 = [rDist(1:2), tDist, rDist(3)];


%% Gathering colour data for undistorted points
r(:, :) = double(frame(1:sR:end, 1:sR:end, 1))/255;
g(:, :) = double(frame(1:sR:end, 1:sR:end, 2))/255;
b(:, :) = double(frame(1:sR:end, 1:sR:end, 3))/255;

cd(:, :, :) = [r(:), g(:), b(:)];

[X, Y] = meshgrid(1:10:v.Width, 1:10:v.Height);
[xd3, yd3] = undistortPoint_mex(X(:), Y(:), camMatrix, distVector3);
[xd6, yd6] = undistortPoint_mex(X(:), Y(:), camMatrix, distVector6);

% 3RD Order Distortion
fig3 = figure;
sp1 = subplot(1, 2, 1);
sp1.Position = [0.05 0.242 0.4 0.516];
s3 = scatter(sp1, xd3, -yd3, [], cd, 'filled', 'SizeData', 2);
axis([-1400, 5400, -2800, 800]);
title("3rd Order Undistortion");
xlabel("Horizontal Pixel Location");
ylabel("Vertical Pixel Location");
sp2 = subplot(1, 2, 2);
sp2.Position = [0.48 0.200 0.48 0.6];
imshow(frame, 'parent', sp2);
title("Original Image");

% 6TH Order Distortion
fig6 = figure;
sp1 = subplot(1, 2, 1);
sp1.Position = [0.05 0.242 0.4 0.516];
s6 = scatter(sp1, xd6, -yd6, [], cd, 'filled', 'SizeData', 2);
axis([-1400, 5400, -2800, 800]);
title("6th Order Undistortion");
xlabel("Horizontal Pixel Location");
ylabel("Vertical Pixel Location");
sp2 = subplot(1, 2, 2);
sp2.Position = [0.48 0.200 0.48 0.6];
imshow(frame, 'parent', sp2);
title("Original Image");