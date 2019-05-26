function rQCc = pix2vec(imgPoint, params)

camMatrix = params.camMatrix;
distVec = params.distVec;
% FOV = param.FOV;

[x, y] = undistortPoint_mex(imgPoint(:, 1), imgPoint(:, 2), camMatrix, distVec);

fovX = params.fovX; %1.2130;
fovY = params.fovY; %2.0630;
resX = params.camResX;
resY = params.camResY;
        
theta = x * (fovX/(resX-1)) - fovX/2;
phi = y * (fovY/(resY-1)) + fovY/4;

rQCc(:, 1) =  cos(theta) .* cos(phi);
rQCc(:, 2) =  cos(theta) .* sin(phi);
rQCc(:, 3) = -sin(theta);

