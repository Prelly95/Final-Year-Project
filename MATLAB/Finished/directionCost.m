function [cost, direction] = directionCost(Gf, eta, targetCost, param)

u(:, :) = Gf(:, :, 1);
v(:, :) = Gf(:, :, 1);

a(:, :) = eta(1, :, :);
b(:, :) = eta(2, :, :);
% Calculate the divergence of the flow vectors
cullDiv = abs(flowDivergence(a, b, u, v));
div = cullDiv.*(cullDiv < param.cutOff*std(std(cullDiv)));

% Normalise the cost tables
divCost = normaliseMatrix(div);
targetCost = normaliseMatrix(targetCost);

% Calculate the cost of moving in that direction
cost = param.flowWeight*divCost - param.devWeight *targetCost;
minCost = min(cost(:));
[row,col] = find(cost == minCost);
direction = eta(:, row(1), col(1));