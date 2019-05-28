function Div = divergence_sphere(X, Y, Z)

dx = diff(X);
dy = diff(Y);
dz = diff(Z);
Div = dx + dy + dz;
