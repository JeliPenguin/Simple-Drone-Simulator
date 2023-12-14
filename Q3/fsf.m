dt = 0.1;
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
        (sin(phi)*sin(psi)*(gamma1+gamma2+gamma3+gamma4)+cos(phi)*cos(psi)*sin(theta)*(gamma1+gamma2+gamma3+gamma4)-0.2*pxd)/m
        (cos(phi)*cos(psi)*sin(theta)*(gamma1+gamma2+gamma3+gamma4)-cos(psi)*sin(phi)*(gamma1+gamma2+gamma3+gamma4)-0.2*pyd)/m
        (cos(phi)*cos(theta)*(gamma1+gamma2+gamma3+gamma4)-0.2*pzd)/m-g
        0.25*(gamma1-gamma3)+0.6*wy*wz
        0.25*(gamma2-gamma4)-0.6*wx*wz
        0.5*(gamma1-gamma2+gamma3-gamma4)
        wx+sin(phi)*tan(theta)*wy+cos(phi)*tan(theta)*wz
        cos(phi)*wy-sin(phi)*wz
        (sin(phi)*wy+cos(phi)*wz)/cos(theta)
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
A = disc_sys.A;
B = disc_sys.B;
assert(rank(ctrb(A,B))==12) % Assert controllability of the system

eigenvalues = [
    0.8
    0.75
    0.76
    0.74
    0.62
    0.63
    0.44
    0.71
    0.73
    0.72
    0.81
    0.7]; % Works pretty well

% eigenvalues = [
%     0.7
%     0.75
%     0.76
%     0.74
%     0.62
%     0.63
%     0.64
%     0.71
%     0.73
%     0.72
%     0.51
%     0.5];

% eigenvalues = rand(12,1)-1/2;
% K = place(A,B,eigenvalues);

Q=eye(12)*0.1;
Q(1,1)=2;
Q(2,2)=2;
Q(3,3)=2;
Q(4,4) = 10;
Q(5,5) = 10;
Q(6,6) = 10;
Q(10,10)=5;
Q(11,11)=5;
Q(12,12)=5;
R=10;
K=dlqr(A,B,Q,R);
