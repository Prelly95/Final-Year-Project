function [g, GT, SQ] = processModel(mu, u, param)
% ProcessModel
% mu    - Current state estimates
% u     - Current plant inputs
% param - model parameters

eta = mu(1:6);
nu = mu(7:end);

