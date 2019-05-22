function dir = easySphere(rohX, rohY, FOV, thX, thY, thZ)

dir = zeros(3, rohX, rohY);

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

for ii = 1:rohX
    for jj = 1:rohY
        
        theta = jj * ((FOV) / (rohX - 1)) - FOV/2;
        phi = ii * ((FOV) / (rohY - 1)) + FOV/2;

        x = [  cos(theta) * cos(phi);
                            cos(theta) * sin(phi);
                            -sin(theta);
                         ];
        
        dir(:, ii, jj) = R*x;
    end
end

% figure;
% quiver3(zeros(30), zeros(30), zeros(30), dir(:, :, 1), dir(:, :, 2), dir(:, :, 3), '.');
