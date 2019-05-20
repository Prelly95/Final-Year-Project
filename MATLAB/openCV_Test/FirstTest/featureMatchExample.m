clc;
clear;
close all;

ref_img = imread('queenOfDiamonds.jpg');
ref_img_gray = rgb2gray(ref_img);

ref_pts = detectSURFFeatures(ref_img_gray);
% [ref_Features, ref_validPts] = extractFeatures(ref_img_gray, ref_pts)

figure;
imshow(ref_img);
hold on;
plot(ref_pts.selectStrongest(50));