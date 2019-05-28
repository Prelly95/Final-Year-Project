function [nu, Map, param] = formatUnityData(lidarData, kinematicData)
% mu = 0;
param.resY = length(lidarData(1, :)) - 1;
ii = 1;
while lidarData(ii, 1) == 0
    ii = ii+1;
end
param.resX = ii-1;

param.frames = min([length(kinematicData), floor(length(lidarData)/param.resY)]) - 1;
Map = zeros(param.resX, param.resY, param.frames);

for ii = 1:param.frames
    for jj = 1:param.resX
        for kk = 1:param.resY
            Map((param.resX+1)-jj, kk, ii) = lidarData((ii-1)*param.resX + jj, kk+1);
        end
    end
end

nu(1:3, :) = kinematicData(:, 2:4)';
nu(4:6, :) = kinematicData(:, 5:7)';