% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 500;
dt          = 0.1;
TIME_SCALE  = 0.1; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


% Initialise plot
figure;
ax1 = axes;
hold(ax1,'on');
% view(ax1, 3);
view(90,0); % for viewing YZ plane
axis('equal')
axis([-5 5 -5 5 0 10])
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

%Observability
C = eye(12);
C(4,4)=0;
C(5,5)=0;
C(6,6)=0;
C(7,7)=0;
C(8,8)=0;
C(9,9)=0;
fsf()
state_observer()

% Drone physics constants
m = 0.3;
g = 9.8;
kd = 0.2;
k = 1;
L = 0.25;
b = 0.2;
I = [1,0,0;0,1,0;0,0,0.4];

% States
p = [0;0;0];
pdot = [0;0;0];
theta=zeros(3,1);
thetadot = zeros(3,1);
x = zeros(12,1);
xhat = zeros(12,1);
simx = zeros(12,1);
operatingGamma = 0.735;

% Reference control
stageNum = 1;
stages = [
    0,0,5,0,0,0,0,0,0,0,0,0
    genCircCheckpoints(5)
    2.5,2.5,2.5,0,0,0,0,0,0,0,0,0
    2.5,2.5,0,0,0,0,0,0,0,0,0,0
];
ref = stages(1,:);
holdTime = 0;
elapseStart = 0;
firstTime = true;
prev = x;

% Run Simulation
for t = 0:dt:TOTAL_TIME
    tic
    cla

    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %

    if stageNum <= 2 || stageNum >= height(stages)-2
        errorThresh = 0.0001;
    else
        % CIrcle trajectory
        errorThresh = 0.01;
    end

    % Adjust references
    if arrivedReference(x,ref,errorThresh)
        if stageNum == 1
            holdTime = 5;
            if firstTime
                elapseStart = t;
                firstTime = false;
            end
            if (t-elapseStart) >= holdTime
                stageNum=min(stageNum+1,height(stages));
            end
        else
            stageNum=min(stageNum+1,height(stages));
        end
        
    end

    ref = transpose(stages(stageNum,:));

    % Add noise
    x = x+rand(size(x)); % noisy measurement

    % FSF controller
    u = min(max((-K*(x-ref))+operatingGamma,0),1.5);
    disp(transpose(u))

    % Switch off quadcopter when completed all stages
    if stageNum >= height(stages)
        u = zeros(4,1);
    end

    %Observer
    


    % disp(calcSpeed(prev,x,dt))

    % Drone physics simulation
    omega = thetadot2omega(thetadot,theta);
    a = acceleration(u,theta,pdot,m,g,k,kd);
    omegadot = angular_acceleration(u,omega,I,L,b,k);
    omega = omega+dt*omegadot;
    thetadot=omega2thetadot(omega,theta);
    theta=theta+dt*thetadot;
    pdot=pdot+dt*a;
    p=p+dt*pdot;
    if p(3) <= 0
        p(3) = 0;
        pdot(3) = 0;
        omega = zeros(3,1);
        theta = zeros(3,1);
    end
    x = [p(1);p(2);p(3);pdot(1);pdot(2);pdot(3);omega(1);omega(2);omega(3);theta(1);theta(2);theta(3)];
    rot=flip(theta);

    drone1.update(p,rot);
    drone1.plot;
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %


    drawnow nocallbacks limitrate
    pause(TIME_SCALE*dt-toc); 
end

function speed=calcSpeed(prev,now,dt)
    s = (now(1:1:3)-prev(1:1:3))/dt;
    speed = sqrt(sum(s.^2));
end

function arrived = arrivedReference(x,ref,errorThresh)
    arrived = all((x-ref).^2<errorThresh);
end

function yCirc = getYFromX(x,neg)
    if neg
        yCirc = sqrt(2.5^2-x^2)+5;
    else
        yCirc = -sqrt(2.5^2-x^2)+5;
    end
    
end

function intermediateCheckpoints = genIntermediateCheckPoints(startState,endState,numPoints)
    diffX = (endState(2)-startState(2));
    step = diffX/(numPoints+1);
    intermediateCheckpoints = [];
    for xCirc = startState(2)+step:step:endState(2)-step
        yCirc = getYFromX(xCirc,step<0);
        newCheckpoint = [0,xCirc,yCirc,0,0,0,0,0,0,0,0,0];
        intermediateCheckpoints = [intermediateCheckpoints;newCheckpoint];
    end

end

function circCheckpoints=genCircCheckpoints(numPointsPerQuad)
    check1 = [0,2.5,5,0,0,0,0,0,0,0,0,0];
    check2 = [0,0,7.5,0,0,0,0,0,0,0,0,0];
    check3 = [0,-2.5,5,0,0,0,0,0,0,0,0,0];
    check4 = [0,0,2.5,0,0,0,0,0,0,0,0,0];
    check5 = [0,2.5,5,0,0,0,0,0,0,0,0,0];
    circCheckpoints = [check1
                       genIntermediateCheckPoints(check1,check2,numPointsPerQuad)
                       check2
                       genIntermediateCheckPoints(check2,check3,numPointsPerQuad)
                       check3
                       genIntermediateCheckPoints(check3,check4,numPointsPerQuad)
                       check4
                       genIntermediateCheckPoints(check4,check5,numPointsPerQuad)
                       check5
                       ];
    % disp(circCheckpoints)
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