clc;
clear;
close all;

% pattern of lixels onto the unit sphere
addpath('./Icosahedron');
[x, y, z, TRI] = make_icosahedron(6, 1, 1);
%%
% setting up the plot
figure();
title("Random Flow");
hold on;
grid on;
view([40, 20]);
axismin = -1;
axismax = 1;
axis([axismin, axismax, axismin, axismax, axismin, axismax]);

trisurf(TRI, x, y, z, 'edgecolor', 'none', 'facecolor', '[0.8, 0.8, 0.8]', 'FaceAlpha', '0.8');
plot1 = quiver3(x, y, z, zeros(size(x)), zeros(size(x)), zeros(size(x)));

% gather paramters
N = length(x);
params.N = N;

frames = 100;

u	= zeros(frames, N);
v	= zeros(frames, N);
w	= zeros(frames, N);

dBrQC	= zeros(3, N);
rPC		= zeros(3, N);

rPC(1, :) = x;
rPC(2, :) = y;
rPC(3, :) = z;

pNoise = perlin2D(2*frames);
vCN = pNoise(1:2:2*frames, 1:3); % Velocity of the camera in world coordinates
omegaBN = pNoise(:, 4:6); % angular velocity of the camera in the world coordinates

%calculate optic flow
for ii = 1:frames
    for jj = 1:N
        [u(ii, jj),  v(ii, jj), w(ii, jj)] = MeasurementModel(rPC(:, jj), vCN(ii, :)', omegaBN(ii, :)');
    end
end

% plot the actual data
for ii = 1:frames
    set(plot1,  'udata', u(ii, :),... 
                'vdata', v(ii, :),...
                'wdata', w(ii, :) ...
       );

    pause(0.2);
end
