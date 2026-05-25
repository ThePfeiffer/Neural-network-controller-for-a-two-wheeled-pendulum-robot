%% init
close all
clear
clc
%%
seed = 5;
rng(seed, 'twister')
%%
open_system('robot_locomotion_RL_VelTurn')

% motor konstants
motor_k = 0.03; % V/(rad/s)
motor_R = 4;
induktance = 13.5e-3;
damping = 0.0003; % Nm/(rad/s)
tyre_fric = 0.1;
% inertia
ankerRadius = 0.0115;
ankerLength = 0.025;
ankerVolumen = ankerRadius^2*pi*ankerLength;
ankerMass = 8000 * ankerVolumen;
ankerInerti = 0.5 * ankerMass * ankerRadius^2;
ankerInerti = ankerInerti * 1.2;
% ankerInerti = 5.5e-6;
%
maxV = 10;
v_delay = 0.015;
%
battery_z_pos = -0.035;
motor_z_pos = -0.022;
%
motor_mass = 0.380;
battery_mass = 0.167;
pcb_mass = 0.098;
total = 1.35;
mass_left = total - 2*motor_mass - pcb_mass - battery_mass;
%
gear = 10 * 84/24; % 19 * 84/24;

robot_width = 0.15 % meter

%%

function x = normRc(mu, sigma, MIN, MAX)
% x = normrnd(mu, sigma) % normally distrubuted number
x = mu + sigma * randn();     % normally distributed number
x = min(max(x, MIN), MAX);    % clamp to range 
x = round(x, 2);
end

% Random mission generator
function [Mission tEnd] = MissionGen()
    % ---- Example: create a new mission each episode ----
    % Random number of waypoints and segment durations
    % Forward,  % backward ,  % clock wise, % counter clock wise
    % fast, slow acceleration % fast, slow decceleration

    dtMin  = 0.5;
    dtMax = 4.5;              % segment duration bounds
    sigma = 0.3;

    tStart = 0.1;

    tStop = 2; %% steady-state = 1.8
    tTurn = 1.8; %% steady-state = 1.2
    tLin = 3; %% steady-state = 1.8

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
    w(4) = tmin + (tmax-tmin) * rand(1,1);      % turn rate rad/s
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
    w(10) = tmin + (tmax-tmin) * rand(1,1);      % rotational rad/s
    t(11) = t(10) + normRc(tTurn, sigma, dtMin, dtMax);
    % reverse turn
    v(11) = 0;
    w(11) = -w(10);
    t(12) = t(11) + normRc(tTurn, sigma, dtMin, dtMax);
    
    nextI = size(t,2);
    % both
    numBoth = 2;
    for i = nextI:nextI+numBoth-1
    v(i) = vtmin + (vtmax-vtmin) * rand(1,1);      % longitudinal m/s
    w(i) = tmin + (tmax-tmin) * rand(1,1);      % rotational rad/s
    t(i+1) = t(i) + normRc(tLin, sigma, dtMin, dtMax);
    end

    v(nextI+numBoth) = 0;
    w(nextI+numBoth) = 0;
    % ------ set output ----------
    Mission = [t', v', w'];
    tEnd = t(end);
    
end
%%
[velStep tEnd] = MissionGen()


function in = localResetFcn(in)
    [Mission tEnd] = MissionGen();
    % ---- Push into the model ----
    in = setVariable(in, 'velStep', Mission);
    in = setVariable(in, 'tEnd', tEnd);

end


%%
% transfer functions for optimal reference.
% velocity:
s = tf('s');
tn = 0.35; %pid 0.35
wn = 1/tn;
zeta = 0.9; %pid 0.9
velFunc = wn^2/(wn^2 + 2*zeta*wn*s + s^2) % 2. order

stepinfo(velFunc)

% tilt
tn = 0.37; %pid 0.37
wn = 1/tn; 
zeta = 0.6; % %pid 0.7
adjust = 0.6; %pid 0.6
tiltFunc = -s*adjust*wn^2/(wn^2 + 2*zeta*wn*s + s^2) % 2. order

tiltVelFunc = tiltFunc*s;
stepinfo(tiltFunc)

% turn rate
s = tf('s');
tn = 0.1;
wn = 1/tn;
zeta = 0.95;
turnFunc = wn^2/(wn^2 + 2*zeta*wn*s + s^2) % 2. order
stepinfo(turnFunc)


figure()
step(velFunc)
hold on;
step(tiltFunc)
step(turnFunc)
hold off

%% Init TD3 agent RL
numObs = 8;
numActs = 2;
layerSize = 31; % max 31 for export
Ts = 0.005;
actionLimit = 10;
LearningRate = 0.01;

obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = "observations";
actionInfo = rlNumericSpec([numActs 1], LowerLimit=-actionLimit, UpperLimit=actionLimit);
actionInfo.Name = "motor voltage";

env = rlSimulinkEnv("robot_locomotion_RL_VelTurn", "robot_locomotion_RL_VelTurn/RL Agent", obsInfo, actionInfo);
env.ResetFcn = @(in) localResetFcn(in);


TD3agent = createTD3Agent(numObs,obsInfo,numActs,actionInfo,Ts, layerSize, actionLimit, LearningRate);

training_start = datetime('now','Format','MM_dd-HH_mm');

trainOpt = rlTrainingOptions( ...
    MaxEpisodes                 = 3000, ...
    MaxStepsPerEpisode          = floor(35*1/Ts), ... % 25 seconds x 1/sampletime
    StopTrainingCriteria        = "none", ...
    ScoreAveragingWindowLength  = 30, ...
    Plots               ="training-progress",...
    UseParallel = false, ... 
    SaveAgentCriteria = "EpisodeReward", ...
    SaveAgentValue = -22000, ...
    SaveAgentDirectory = "savedAgents/TD3_" + string(training_start) + "_l"+string(layerSize), ...
    SimulationStorageType = "file")

actor = getActor(TD3agent);
model = getModel(actor);
%% train
trainResults = train(TD3agent,env,trainOpt)

%%
filename = "savedAgents/TD3_" + string(datetime('now','Format','MM_dd-HH_mm')) + "_ar" + string(floor(trainResults.AverageReward(end))) + "_l"+string(layerSize) + ".mat"
save(filename, "TD3agent", "trainResults")


%%
load('savedAgents/TD3_05_12-18_33_l31/Agent2043.mat')
figure()
hold on
plot(savedAgentResult.EpisodeReward)
plot(savedAgentResult.AverageReward)
plot(savedAgentResult.EpisodeQ0)
xlim([0, savedAgentResult.EpisodeIndex(end)])
ylim([-8000, 0])
hold off

[m, i] = max(savedAgentResult.AverageReward)

%%
load('savedAgents/TD3_05_12-18_33_l31/Agent358.mat')


%%
load("RL_VelTurn/Agent358.mat")
model = getModel(getActor(saved_agent)) % TD3
plot(model);

model = removeLayers(model, "scale")
model = removeLayers(model, "tanh")
plot(model);

exportNetworkToTensorFlow(model, "TD3modelVelTurn")

