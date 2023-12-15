% Drone physics simulation
omega = thetadot2omega(thetadot,theta);
a = acceleration(u,theta,pdot,m,g,k,kd);
omegadot = angular_acceleration(u,omega,I,L,b,k);
omega = omega+dt*omegadot;
thetadot=omega2thetadot(omega,theta);
theta=theta+dt*thetadot;
pdot=pdot+dt*a;
p=p+dt*pdot;

% Ground physics
if p(3) <= 0
    p(3) = 0;
    pdot(3) = 0;
    omega = zeros(3,1);
    theta = zeros(3,1);
end

x = [p(1);p(2);p(3);pdot(1);pdot(2);pdot(3);omega(1);omega(2);omega(3);theta(1);theta(2);theta(3)];
rot = flip(theta);


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

function T = thrust(inputs,k)
    T=[0;0;k*sum(inputs)];
end

function tau=torques(inputs,L,b,k)
    tau=[L*k*(inputs(1)-inputs(3))
        L*k*(inputs(2)-inputs(4))
        b*(inputs(1)-inputs(2)+inputs(3)-inputs(4))];
end