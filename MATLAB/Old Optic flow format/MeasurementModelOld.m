function [u, v, w] = MeasurementModelOld(rQC, rPC, vCN, omegaBN)

if(norm(rPC) == 0)
    rhoPC = 0;
%     rQC = [0;0;0];
else
    rhoPC = 1/norm(rPC);
    rQC = rPC/norm(rPC);
end

dBrQC = skew(rQC)*[1*skew(rQC), eye(3)]*[vCN; omegaBN];


u = dBrQC(1);
v = dBrQC(2);
w = dBrQC(3);