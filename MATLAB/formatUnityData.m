function [nu, Map, param] = formatUnityData(lidarData, kinematicData)
% mu = 0;
param.resX = length(lidarData(1, :)) - 1;
ii = 1;
while lidarData(ii, 1) == 0
    ii = ii+1;
end
param.resY = ii-1;

param.frames = min([length(kinematicData), floor(length(lidarData)/param.resY)]);
Map = zeros(param.frames, param.resX, param.resY);

for ii = 1:param.frames
    for jj = 1:param.resX
        for kk = 1:param.resY
            Map(ii, kk, (param.resX+1)-jj) = lidarData((ii-1)*param.resX + jj, kk+1);
        end
    end
end

nu(1:3, :) = kinematicData(:, 2:4)';
nu(4:6, :) = kinematicData(:, 5:7)';