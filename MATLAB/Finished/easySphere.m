function dir = easySphere(camAngle, params)

thX = camAngle(1);
thY = camAngle(2);
thZ = camAngle(3);

resX = params.resX;
resY = params.resY;

FOV = params.FOV;

dir = zeros(3, resX, resY);

thX = deg2rad(thX);
thY = deg2rad(thY);
thZ = deg2rad(thZ);

Rx = [   1,  0,         0
         0,  cos(thX),	-sin(thX)
         0,  sin(thX),  cos(thX)];
     
Ry = [   cos(thY),	0,	sin(thY)
         0,           1,  0
         -sin(thY),	0,  cos(thY)];
     
Rz = [   cos(thZ),    -sin(thZ),    0
         sin(thZ),    cos(thZ),     0
         0,           0,             1];

R = Rx*Ry*Rz;

for ii = 1:resX
    for jj = 1:resY
        
        theta = jj * ((FOV) / (resY - 1)) - FOV/2;
        phi = ii * ((FOV) / (resX - 1)) + FOV/2;

        x = [  cos(theta) * cos(phi);
                            cos(theta) * sin(phi);
                            -sin(theta);
                         ];
        
        dir(:, ii, jj) = R*x;
    end
end

% figure;
% quiver3(zeros(30), zeros(30), zeros(30), dir(:, :, 1), dir(:, :, 2), dir(:, :, 3), '.');
