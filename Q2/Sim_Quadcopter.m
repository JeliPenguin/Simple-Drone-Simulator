% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 100;
dt          = 0.1;
TIME_SCALE  = 0.1; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


% Initialise plot
figure;
ax1 = axes;
hold(ax1,'on');
view(ax1, 3);
axis('equal')
axis([-20 5 -5 5 0 10])
axis('manual')
xlabel('x');
ylabel('y');
ylabel('z');
axis vis3d
grid ON
grid MINOR
ax1.Toolbar.Visible = 'off';
ax1.Interactions = [];

% Initialise Simulation
drone1 = Quadcopter(ax1);

m = 0.3;
g = 9.8;
kd = 0.2;
k = 1;
L = 0.25;
b = 0.2;
I = [1,0,0;0,1,0;0,0,0.4];
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
% C = eye(12);
% D = zeros(size(B));
% cont_sys = ss(A,B,C,D);
% disc_sys = c2d(cont_sys,dt,'zoh');
% A = disc_sys.A;
% B = disc_sys.B;
x = [0;0;3;0;0;0;0;0;0;0;0;0];
p = [0;0;3];
u = zeros(4,1);

% Run Simulation
for t = 0:dt:TOTAL_TIME
    tic
    cla

    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %

    % xdot = A*x+B*u;
    % x = x + dt*xdot;

    % Equilibrium Case
    x = expm(A*dt)*x;

    % % Small error case
    % x = x -0.00001;

    % Large error case
    x = x -0.1;

    if p(3)<=0
        p(3)=0;
        theta = zeros(3,1);
    else
        p = [x(1),x(2),x(3)];
        theta = [x(12),x(11),x(10)];
    end

    disp(p)

    drone1.update(p,theta);
    drone1.plot;
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %


    drawnow nocallbacks limitrate
    pause(TIME_SCALE*dt-toc); 
end