function Div = flowDivergence(x, y, u, v)
%Divergence Aproximation.
%   Calculates the divergence of the vector field u, u with respect to x, y
%   Does the same thing as divergence(x, y, u, v) without some of the
%   overhead

u = double(u);
v = double(v);

w = x(1,:); 
h = y(:,1); 

[dx, ~] = gradient(u, w, h);
[~, dy] = gradient(v, w, h);

Div = dx + dy;