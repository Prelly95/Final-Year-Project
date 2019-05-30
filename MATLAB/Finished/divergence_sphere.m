function Div = divergence_sphere(hx, hy, u, v)

u = double(u);
u = double(u);

[px, ~] = gradient(u, hx, hy); 
[~, qy] = gradient(v, hx, hy); 

Div = px + qy;