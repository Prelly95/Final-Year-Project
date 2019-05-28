clc;
clear;
close all;

addpath('./Icosahedron');

[x, y, z, TRI] = make_icosahedron(6, 1, 1);

N = length(x);
params.N = N;

frames = 1;

u	= zeros(frames, N);
v	= zeros(frames, N);
w	= zeros(frames, N);

dBrQC	= zeros(3, N);
rPC		= zeros(3, N);

rPC(1, :) = x;
rPC(2, :) = y;
rPC(3, :) = z;

vCN = [1; 0; 0]; % Velocity of the camera in world coordinates
omegaBN = [0; 0; 0]; % angular velocity of the camera in the world coordinates

for jj = 1:N
    G(:, jj) = MeasurementModel([vCN;omegaBN], rPC(:, jj), 1);
end

figure;
hold on;
grid on;
quiver3(x, y, z, G(1, :), G(2, :), G(3, :));
trisurf(TRI, x, y, z, 'edgecolor', 'none', 'facecolor', '[0.8, 0.8, 0.8]', 'FaceAlpha', '0.8');
view([40, 20]);
title("Unit Surge");
