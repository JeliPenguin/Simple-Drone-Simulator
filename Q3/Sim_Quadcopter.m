% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 1000;
dt          = 0.1;
TIME_SCALE  = 0.1; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


% Initialise plot
figure;
ax1 = axes;
hold(ax1,'on');
view(ax1, 3);
% view(90,0); % for viewing YZ plane
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

% Drone physics constants
m = 0.3;
g = 9.8;
kd = 0.2;
k = 1;
L = 0.25;
b = 0.2;
I = [1,0,0;0,1,0;0,0,0.4];

C = eye(12);
fsf() % Calculates K matrix

% States
p = [0;0;0];
pdot = [0;0;0];
theta=zeros(3,1);
thetadot = zeros(3,1);
x = zeros(12,1);
xhat = zeros(12,1);
u = zeros(4,1);
simx = zeros(12,1);
operatingGamma = 0.735;

% Reference control
stageNum = 1;
circCheckPoints = genCircCheckpoints(30);
circCheckPointsLen = height(circCheckPoints);
landingCheckPoints = genLineCheckpoints(-0.1,dt);
landingCheckPointsLen = height(landingCheckPoints);
stages = [
    0,0,5,0,0,0,0,0,0,0,0,0
    circCheckPoints;
    1.25,2.5,5,0,0,0,0,0,0,0,0,0
    1.25,2.5,2.5,0,0,0,0,0,0,0,0,0
    landingCheckPoints;
];

ref = stages(1,:);
holdTime = 0;
elapseStart = 0;
firstTime = true;
prev = x;

% For plotting all reference points
% plot3(ax1,stages(:,1),stages(:,2),stages(:,3),'.','MarkerSize',20)


allStates=[];
stageCompletionStep = [];
step = 1;
titles = ["X","Y","Z","X dot","Y dot","Z dot","Omega X","Omega Y","Omega Z","Phi","Theta","Psi"];

% Run Simulation
for t = 0:dt:TOTAL_TIME
    tic
    cla

    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %


    if stageNum <= 2 || stageNum >= height(stages)-2
        errorThresh = 0.0001;
    else
        % Circle trajectory
        errorThresh = 0.001;
    end

    % Adjust references
    if arrivedReference(x,ref,errorThresh)
        if stageNum == 1
            holdTime = 5;
            if firstTime
                stageCompletionStep=[step];
                elapseStart = t;
                firstTime = false;
            end
            if (t-elapseStart) >= holdTime
                stageNum=stageNum+1;
                stageCompletionStep=[stageCompletionStep,step];
            end
        else
            stageNum=min(stageNum+1,height(stages));
            if stageNum <= 5+circCheckPointsLen
                stageCompletionStep=[stageCompletionStep,step];
            end
        end 
    end

    ref = transpose(stages(stageNum,:));

    % For plotting
    allStates= [allStates;transpose(x)];
    step = step+1;

    % FSF controller
    u = min(max((-K*(x-ref))+operatingGamma,0),1.5);

    % Switch off quadcopter when completed all stages
    if stageNum >= height(stages)
        u = zeros(4,1);
        close all
        break
    end

    drone_physics() % Gives new state according to drone physics simulation

    drone1.update(p,rot);
    drone1.plot;
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %


    drawnow nocallbacks limitrate
    pause(TIME_SCALE*dt-toc); 
end

stages = [0,0,5,0,0,0,0,0,0,0,0,0;stages];
[r,c] = size(allStates); % get number of rows and columns of A
for index = 1:c % columns 1 to c
    f=figure(index);
    plot(allStates(:,index))
    hold on
    % xline(stageCompletionStep,"--r")
    plot(stageCompletionStep,stages(1:(5+circCheckPointsLen),index),'.','MarkerSize',20)
    title(titles(index));
    xlabel("Time step")
    grid on
    saveas(f,"plots/"+titles(index),"fig")
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

function lineCheckpoints = genLineCheckpoints(velocity,dt)
    lineCheckpoints = [2.5,2.5,2.5,0,0,0,0,0,0,0,0,0];
    startPoint = 2.5;
    endPoint = 0;
    step=velocity*dt;
    for i=startPoint+step:step:endPoint-step
        newPoint = [2.5,2.5,i,0,0,velocity,0,0,0,0,0,0];
        lineCheckpoints = [lineCheckpoints;newPoint];
    end
    lineCheckpoints = [lineCheckpoints;[2.5,2.5,0,0,0,0,0,0,0,0,0,0]];
end