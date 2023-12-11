m = 0.3;
g = 9.8;
kd = 0.2;
k = 1;
L = 0.25;
b = 0.2;
syms px py pz pxd pyd pzd wx wy wz phi theta psi gamma1 gamma2 gamma3 gamma4
xC = [px;py;pz;pxd;pyd;pzd;wx;wy;wz;phi;theta;psi];
uC = [gamma1;gamma2;gamma3;gamma4];

xdotC = [pxd
        pyd
        pzd
        (cos(phi)*sin(psi)*(gamma1+gamma2+gamma3+gamma4)+cos(phi)*cos(psi)*sin(theta)*(gamma1+gamma2+gamma3+gamma4)-0.2*pxd)/m
        (cos(phi)*cos(psi)*sin(theta)*(gamma1+gamma2+gamma3+gamma4)-cos(psi)*sin(phi)*(gamma1+gamma2+gamma3+gamma4)-0.2*pyd)/m
        (cos(phi)*cos(theta)*(gamma1+gamma2+gamma3+gamma4)-0.2*pzd)/m-g
        0.25*(gamma1-gamma3)-0.6*wy*wz
        0.25*(gamma2-gamma4)+0.6*wx*wz
        0.5*(gamma1-gamma2+gamma3-gamma4)
        wx+sin(phi)*tan(theta)*wy+cos(phi)*tan(theta)*wz
        cos(phi)*wy-sin(phi)*wz
        sin(phi)*wy/cos(theta)+cos(phi)*wz/cos(theta)
        ];

Aj = jacobian(xdotC,xC);
Bj = jacobian(xdotC,uC);

operatingGamma = 0.735;
Aj = subs(Aj,[phi,theta,psi,wx,wy,wz,gamma1,gamma2,gamma3,gamma4],[0,0,0,0,0,0,operatingGamma,operatingGamma,operatingGamma,operatingGamma]);
A = double(Aj);
Bj = subs(Bj,[phi,theta,psi],[0,0,0]);
B = double(Bj);
C = eye(12);
D = zeros(size(B));
cont_sys = ss(A,B,C,D);
disc_sys = c2d(cont_sys,dt,'zoh');
assert(rank(ctrb(disc_sys.A,disc_sys.B))==12) % Assert controllability of the system

% eigenvalues =[0.02,0.03,0.04,0.1,0.2,0.3,0.4,0.5,0.6,0.75,0.8,0.9];
eigenvalues = [-0.3485
    0.4962
   -0.1631
   -0.4108
    0.1759
    0.1652
    0.4437
    0.2183
    0.3343
    0.0388
   -0.4217
   -0.3079];
K = place(disc_sys.A,disc_sys.B,eigenvalues);
