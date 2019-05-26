function rQCc = pix2vec(imgPoint, params, undistort)

camMatrix = params.camMatrix;
distVec = params.distVec;

if(undistort == 1)
    [x, y] = undistortPoint_mex(imgPoint(:, 1), imgPoint(:, 2), camMatrix, distVec);
else
    x = imgPoint(:, 1);
    y = imgPoint(:, 2);
end

fovX = params.fovX; %2.0630;
fovY = params.fovY; %1.2130;
resX = params.camResX;
resY = params.camResY;
        
phi = x * (fovX/(resX-1))+ fovX/4;
theta = y * (fovY/(resY-1)) - fovY/2;

rQCc(:, 1) =  cos(theta) .* cos(phi);
rQCc(:, 2) =  cos(theta) .* sin(phi);
rQCc(:, 3) = -sin(theta);

