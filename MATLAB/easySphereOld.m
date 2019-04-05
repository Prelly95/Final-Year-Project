function [x, y, z] = easySphereOld(roh, FOV, thX, thY, thZ)

x = zeros(1, roh);
y = zeros(1, roh);
z = zeros(1, roh);
k= 1;

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

for ii = 1:roh
    for jj = 1:roh
        
        theta = jj * ((FOV) / (roh - 1)) - FOV/2;
        phi = ii * ((FOV) / (roh - 1)) + FOV/2;

        dir = [  cos(theta) * cos(phi)
                    cos(theta) * sin(phi)
                    -sin(theta)
                 ];
        
        dir = R*dir;
        
        x(k) = dir(1);
        y(k) = dir(2);
        z(k) = dir(3);
        
        k=k+1;
    end
end

% figure;
% plot3(x, y, z, '.');
% view([0, 0]);
