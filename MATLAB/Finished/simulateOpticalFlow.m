function dBrQC = simulateOpticalFlow(nu, eta, M, params)

sigX  = params.sigX;
sigY = params.sigY;

noise = zeros(3, 1);

noise(1) = sigX*randn(1);
noise(2) = sigY*randn(1);

vCN = nu(1:3);
omegaBN = nu(4:6);

rQC = eta;

Y = null([rQC(1), rQC(3), rQC(2)]);

R = [Y, [rQC(1); rQC(3); rQC(2)]];

if(M == 0)
    rhoPC = 0;
else
    rhoPC = 1/norm(rQC*M);
end

% nT = RTS*noise;
dBrQC = skew(rQC)*[rhoPC*skew(rQC), eye(3)]*[vCN; omegaBN];

yTS = R'*dBrQC + noise;
dBrQC = R*yTS;

