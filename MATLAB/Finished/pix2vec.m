function rQCc = pix2vec(imgPoint, params, undistort)

if(undistort == 1)
    camMatrix = params.camMatrix;
    distVec = params.distVec;
    [x, y] = undistortPoint_mex(imgPoint(:, 1), imgPoint(:, 2), camMatrix, distVec);
else
    x = imgPoint(:, 1);
    y = imgPoint(:, 2);
end

fovX = params.fovX; %
fovY = params.fovY; %1.2130;
resX = params.resX;
resY = params.resY;
        
phi = x * (fovX/(resX-1)) + (pi/2 - fovX/2);
theta = y * (fovY/(resY-1)) - fovY/2;

rQCc(:, 1) =  cos(theta) .* cos(phi);
rQCc(:, 2) =  cos(theta) .* sin(phi);
rQCc(:, 3) = -sin(theta);

