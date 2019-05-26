function dBrQC = MeasurementModel(nu, eta, M)

vCN = nu(1:3);
omegaBN = nu(4:6);

rQC = eta;

if(M == 0)
    rhoPC = 0;
else
    rhoPC = 1/norm(rQC*M);
end

dBrQC = skew(rQC)*[rhoPC*skew(rQC), eye(3)]*[vCN; omegaBN];
  