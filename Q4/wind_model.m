u20 = 0.0001;
wC = [
  u20*log(-pz/0.15)/log(20/0.15)
  0
  0
];

Lw = -pz;
Lv = -pz/(0.177+0.000823*-pz)^1.2;
Lu = Lv;

sigW = 0.1*u20;
sigU=sigW/(0.177+0.000823*-pz)^0.4;
sigV = sigU;

epsU = rand(1)*sigU;
epsV = rand(1)*sigV;
epsW = rand(1)*sigW;
wT = [
    (1-V*dt/Lu)*wU + sqrt(2*V*dt/Lu)*epsU
    (1-V*dt/Lv)*wV + sqrt(2*V*dt/Lv)*epsV
    (1-V*dt/Lw)*wW + sqrt(2*V*dt/Lw)*epsW
];

wU = wT(1);
wV = wT(2);
wW = wT(3);

windVec = wC + wT;
