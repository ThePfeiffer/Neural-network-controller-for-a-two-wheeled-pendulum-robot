%% init
close all
clear
clc
%%
seed = 5;
rng(seed, 'twister');
load('TunedPIDworkspace.mat')
open(['robot_locomotion_SL_VelTurn.slx'])

%% Random mission generator
function x = normRc(mu, sigma, MIN, MAX)
x = mu + sigma *randn();     % normally distributed number
x = min(max(x, MIN), MAX);    % clamp to range
x = round(x, 2);
end

% Random mission generator
function [Mission tEnd] = MissionGen()
    % ---- Example: create a new mission each episode ----
    % Random number of waypoints and segment durations
    % Forward,  % backward ,  % clock wise, % counter clock wise
    % fast, slow acceleration % fast, slow decceleration
    
    % segment duration bounds
    dtMin  = 2;
    dtMax = 2.7;              
    sigma = 1;

    tStart = 0.1;
    
    % tStop = 1; %% steady-state = 1.8
    % tTurn = 0.6; %% steady-state = 1.2
    % tLin = 1; %% steady-state = 1.8
    
    tStop = 1.8; %% steady-state = 1.8
    tTurn = 1.2; %% steady-state = 1.2
    tLin = 1.8; %% steady-state = 1.8

    % Create velocity profiles. Customize to your use case:
    vlmin = -0.5; vlmax = -vlmin; % without saturation = 0.5
    vtmin = -0.35; vtmax = -vtmin; % without saturation = 0.35
    tmin = -2.1; tmax = -tmin; % without saturation = 2.1
    
    % initialize
    v = 0;
    w = 0;
    t = 0;
    t(2) = tStart;

    % only linear
    v(2) = vlmin + (vlmax-vlmin) * rand(1,1);      % longitudinal m/s
    w(2) = 0;
    t(3) = t(2) + normRc(tLin, sigma, dtMin, dtMax);

    %stop
    v(3) = 0;
    w(3) = 0;
    t(4) = t(3) + normRc(tStop, sigma, dtMin, dtMax);

    % only turn
    v(4) = 0;
    w(4) = tmin + (tmax-tmin) * rand(1,1);      % Heading rad/s
    t(5) = t(4) + normRc(tTurn, sigma, dtMin, dtMax);

    %stop
    v(5) = 0;
    w(5) = 0;
    t(6) = t(5) + normRc(tStop, sigma, dtMin, dtMax);

    % both
    v(6) = vtmin + (vtmax-vtmin) * rand(1,1);      % longitudinal m/s
    w(6) = tmin + (tmax-tmin) * rand(1,1);      % rotational rad/s
    t(7) = t(6) + normRc(tLin, sigma, dtMin, dtMax);

    %stop 
    v(7) = 0;
    w(7) = 0;
    t(8) = t(7) + normRc(tStop, sigma, dtMin, dtMax);

    % only linear
    v(8) = vlmin + (vlmax-vlmin) * rand(1,1);      % longitudinal m/s
    w(8) = 0;
    t(9) = t(8) + normRc(tLin, sigma, dtMin, dtMax);
    %reverse
    v(9) = -v(8);
    w(9) = 0;
    t(10) = t(9) + normRc(tLin, sigma, dtMin, dtMax);

    % only turn
    v(10) = 0;
    w(10) = tmin + (tmax-tmin) * rand(1,1);      % Heading rad/s
    t(11) = t(10) + normRc(tTurn, sigma, dtMin, dtMax);
    % reverse turn
    v(11) = 0;
    w(11) = -w(10);
    t(12) = t(11) + normRc(tTurn, sigma, dtMin, dtMax);
    
    % both
    numBoth = 3;
    for i = 12:12+numBoth-1
    v(i) = vtmin + (vtmax-vtmin) * rand(1,1);      % longitudinal m/s
    w(i) = tmin + (tmax-tmin) * rand(1,1);      % rotational rad/s
    t(i+1) = t(i) + normRc(tLin, sigma, dtMin, dtMax);
    end

    v(12+numBoth) = 0;
    w(12+numBoth) = 0;
    % ------ set output ----------
    Mission = [t', v', w'];
    tEnd = t(end);
    
end

[velStep tEnd] = MissionGen()

%%
[velStep tEnd] = MissionGen()

%% generate training missions
nMis = 3000;

Missions = [];
Ends = [];
for i=1:nMis
    [velStep tEnd] = MissionGen();
    Missions = [Missions; velStep];
    Ends(i) = tEnd;
end
nSeg = size(velStep,1);
filename = 'Missions_'+string(datetime('now','Format','yyyy_MM_dd-HH_mm'))+'_n'+string(nMis)+'_s'+string(seed)+'.mat'
save(filename, 'Missions', 'Ends','nSeg');


%% generate training data, parallel
load("Missions_2026_05_09-11_48_n1000_s5.mat")
nMis = length(Ends);

simIn(1:nMis) = Simulink.SimulationInput('robot_locomotion_invTest');

for i = 1:nMis
    velStep = Missions((nSeg*(i-1)+1:nSeg*i), :);
    tEnd = Ends(i);

    simIn(i) = simIn(i).setVariable('velStep', velStep);
    simIn(i) = simIn(i).setVariable('tEnd', tEnd);
end

%simulate
simOut = parsim(simIn, 'UseFastRestart', 'on', "TransferBaseWorkspaceVariables", 'on');

%store results
trainIn  = [];
trainOut = [];
trainTime = [];
for i = 1:nMis
    trainIn  = [trainIn;  simOut(i).RobotInLeft.data simOut(i).RobotInRight.data];
    trainOut = [trainOut; simOut(i).RobotOutLeft.data ...
                           simOut(i).RobotOutRight.data ...
                           simOut(i).RobotOutVel.data ...
                           simOut(i).RobotOutTurn.data ...
                           simOut(i).RobotOutTiltVel.data ...
                           simOut(i).RobotOutTiltAngle.data ...
                           simOut(i).Vref.data ...
                           simOut(i).Wref.data ...
                           simOut(i).RobotOutH.data ...
                           simOut(i).RobotOutX.data];
    trainTime = [trainTime; simOut(i).RobotInLeft.time];
end
filename = 'training_data_'+string(datetime('now','Format','yyyy_MM_dd-HH_mm'))+'_n'+string(nMis)+'.mat';
save(filename, 'trainIn', 'trainOut', 'trainTime');



%% train NN - DL net - parallel

load('training_data_2026_05_09-12_01_n1000.mat')
nIn = 8;
nOut = 2;

trainO = [trainOut(:,1:8)];
trainI = trainIn;

% Define a shallow MLP using deep-learning layers
layerSize = 31;

Layers = [
    featureInputLayer(nIn)
    fullyConnectedLayer(layerSize, Name="FC1")
    reluLayer(Name="Relu1")
    fullyConnectedLayer(layerSize, Name="FC2")
    reluLayer(Name="Relu2")
    fullyConnectedLayer(nOut, Name="FC3") 
    tanhLayer(Name = 'tanh')
    scalingLayer(Name="scale",Scale = 10)
    ];

% Convert to dlnetwork object and initialize
Network = initialize(dlnetwork(Layers));

trainOpts = trainingOptions("adam", ...
    ExecutionEnvironment="parallel-auto", ...
    Plots="training-progress");
    
%%
% Train using MSE loss
[net_trained, info] = trainnet(trainO, trainI, Network, "mse", trainOpts);

filename = "trained_invDLNN_" + string(datetime('now','Format','yyyy_MM_dd-HH_mm')) + "-l"+ string(layerSize) + ".mat";
save(filename, 'net_trained', 'info');

%% Generate Simulink block of trained NN
load 'trained_invDLNN_2026_05_11-12_01-l31.mat'
%%
exportNetworkToSimulink(net_trained, "modelName", "Data_Net_8_31_31_2")
