function dst = unityReadDist(resX, resY, lData)
% Reads the ray cast distance data from unity into a matlab matrix with the
% correct orrientation
    % resX: Number of vectors in the x direction
    % resY: Number of vectors in the y direction
    
dst = zeros(resX, resY);

for ii = 1:resX
    for jj = 1:resY
        dst(jj, (resX+1)-ii) = lData(ii, jj+1);
    end
end