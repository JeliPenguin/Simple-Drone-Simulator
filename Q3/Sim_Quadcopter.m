% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 20;
dt          = 0.1;
TIME_SCALE  = 1; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


% Initialise plot
figure;
ax1 = axes;
hold(ax1,'on');
view(ax1, 3);
axis('equal')
axis([-5 5 -5 5 0 5])
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
fsf()

m = 0.3;
g = 9.8;
kd = 0.2;
k = 1;
L = 0.25;
b = 0.2;
I = [1,0,0;0,1,0;0,0,0.4];
p = [0;0;0];
pdot = [0;0;0];
theta=zeros(3,1);
thetadot = zeros(3,1);
x = [0;0;0;0;0;0;0;0;0;0;0;0];
ref = [0;0;1;0;0;0;0;0;0;0;0;0];
% Run Simulation
for t = 0:dt:TOTAL_TIME
    tic
    cla

    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    
    % FSF controller
    u = K*(x-ref);
    disp(x-ref)
    u = min(max(u,1.5),1.5);
    x = (A-B*K)*x+B*K*ref;

    % Drone simulation
    omega = thetadot2omega(thetadot,theta);
    a = acceleration(u,theta,pdot,m,g,k,kd);
    omegadot = angular_acceleration(u,omega,I,L,b,k);
    omega = omega+dt*omegadot;
    thetadot=omega2thetadot(omega,theta);
    theta=theta+dt*thetadot;
    pdot=pdot+dt*a;
    p=p+dt*pdot;
    omegaFlip=flip(omega);
    disp(p)

    drone1.update(p,omegaFlip);
    drone1.plot;
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %


    drawnow nocallbacks limitrate
    pause(TIME_SCALE*dt-toc); 
end

function T = thrust(inputs,k)
    T=[0;0;k*sum(inputs)];
end

function tau=torques(inputs,L,b,k)
    tau=[L*k*(inputs(1)-inputs(3))
        L*k*(inputs(2)-inputs(4))
        b*(inputs(1)-inputs(2)+inputs(3)-inputs(4))];
end

function R=rotation(angles)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    R=[cos(psi)*cos(theta),cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi),sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)
        cos(theta)*sin(psi),cos(phi)*cos(psi)+sin(phi)*sin(psi)*sin(theta),cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi)
        -sin(theta),cos(theta)*sin(phi),cos(phi)*cos(theta)
        ];
end

function a = acceleration(inputs,angles,xdot,m,g,k,kd)
    gravity=[0;0;-g];
    R = rotation(angles);
    T = R*thrust(inputs,k);
    Fd = -kd*xdot;
    a = gravity+T/m+Fd/m;
end

function omegadot = angular_acceleration(inputs,omega,I,L,b,k)
    tau = torques(inputs,L,b,k);
    omegadot = inv(I)*(tau-cross(omega,I*omega));
    % Ixx = I(1,1);
    % Iyy = I(2,2);
    % Izz = I(3,3);
    % omegadot = [tau(1)/Ixx;tau(2)/Iyy;tau(3)/Izz] - [(Iyy-Izz)*omega(2)*omega(3)/Ixx;(Izz-Ixx)*omega(1)*omega(3)/Iyy;(Ixx-Iyy)*omega(1)*omega(2)/Izz];
end

function thetadot=omega2thetadot(omega,angle)
    phi = angle(1);
    theta = angle(2);
    thetadot=inv([1,0,-sin(theta);0,cos(phi),cos(theta)*sin(phi);0,-sin(phi),cos(theta)*cos(phi)])*omega;
end

function omega=thetadot2omega(thetadot,angle)
    phi = angle(1);
    theta = angle(2);
    omega=[1,0,-sin(theta);0,cos(phi),cos(theta)*sin(phi);0,-sin(phi),cos(theta)*cos(phi)]*thetadot;
end