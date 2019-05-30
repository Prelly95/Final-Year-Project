function M = normaliseMatrix(N)

t = max(max(N));
b = min(min(N));
if(t-b ~= 0)
    M = (N-b)/(t-b);
else
    error("Dividing by zero");
end
    