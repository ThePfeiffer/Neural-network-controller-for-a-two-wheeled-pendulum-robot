%% init
close all
clear
clc
%%
seed = 5;
rng(seed, 'twister')
%%
open_system('robot_locomotion_RL_VelDual')

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
% range limited Gaussian
function x = normRc(mu, sigma, MIN, MAX)
x = mu + sigma * randn();     % normally distributed number
x = min(max(x, MIN), MAX);    % clamp to range 
x = round(x, 2);
end

% Random mission generator
function [Mission tEnd] = VelMissionGen()
    % ---- Example: create a new mission each episode ----
    % Random number of waypoints and segment durations
    % Forward,  % backward ,  % clock wise, % counter clock wise
    % fast, slow acceleration % fast, slow decceleration

    tStart = 0.1;
    tLin = 5; %% steady-state = 1.8 for PID
    % segment duration bounds
    dtMin  = 1.5;
    dtMax = 6;              
    sigma = 0.2;

    % Create velocity profiles. Customize to your use case:
    vlmin = -0.5; vlmax = -vlmin; % without saturation = 0.5
    
    % initialize
    v = 0;
    t = 0;
    t(2) = tStart;
    
    % longer time to settle
    nextI = size(t,2);
    num = 4;
    for i = nextI:nextI+num-1
    v(i) = vlmin + (vlmax-vlmin) * rand(1,1);      % longitudinal m/s
    t(i+1) = t(i) + normRc(tLin, sigma, dtMin, dtMax);
    end
    
    % shorter time to settle
    tLin = 1; %% steady-state = 1.8 for PID
    
    nextI = size(t,2);
    num = 4;
    for i = nextI:nextI+num-1
    v(i) = vlmin + (vlmax-vlmin) * rand(1,1);      % longitudinal m/s
    t(i+1) = t(i) + normRc(tLin, sigma, dtMin, dtMax);
    nextI = size(t,2);
    end
    
    %stop
    v(nextI) = 0;
    t(nextI+1) = t(nextI)+3; % normRc(tLin, sigma, dtMin, dtMax);
    v(nextI+1) = 0;
    
    w = zeros(1, nextI+1); % turnrate reference for PID
    % ------ set output ----------
    Mission = [t', v', w'];
    tEnd = t(end);
    
end

[velStep tEnd] = VelMissionGen()
%%
% reset function, for new mission every training
function in = localResetFcn(in)
    [Mission tEnd] = VelMissionGen();
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
zeta = 0.7; % %pid 0.7
adjust = 0.6; %pid 0.6
tiltFunc = -s*adjust*wn^2/(wn^2 + 2*zeta*wn*s + s^2) % 2. order

tiltVelFunc = tiltFunc*s;
stepinfo(tiltFunc)

% turnrate
s = tf('s');
tn = 0.15;
wn = 1/tn;
zeta = 0.95;
turnFunc = wn^2/(wn^2 + 2*zeta*wn*s + s^2) % 2. order
stepinfo(turnFunc)

% figure()
% step(velFunc)
% hold on;
% step(tiltFunc)
% % step(tiltVelFunc)
% hold off
% xlim([0, 10])
% ylim([-0.9, 1.1])

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

env = rlSimulinkEnv("robot_locomotion_RL_VelDual", "robot_locomotion_RL_VelDual/RL Agent", obsInfo, actionInfo);
env.ResetFcn = @(in) localResetFcn(in);

TD3agent = createTD3Agent(numObs,obsInfo,numActs,actionInfo,Ts, layerSize, actionLimit, LearningRate);

training_start = datetime('now','Format','MM_dd-HH_mm');

trainOpt = rlTrainingOptions( ...
    MaxEpisodes                 = 3000, ...
    MaxStepsPerEpisode          = floor(25*1/Ts), ... % 25 seconds x 1/sampletime
    StopTrainingCriteria        = "none", ...
    ScoreAveragingWindowLength  = 30, ...
    Plots               ="training-progress",...
    UseParallel = false, ... 
    SaveAgentCriteria = "EpisodeReward", ...
    SaveAgentValue = -12000, ...
    SaveAgentDirectory = "savedAgents/Simple_2_TD3_" + string(training_start) + "_l"+string(layerSize), ...
    SimulationStorageType = "file")

actor = getActor(TD3agent);
model = getModel(actor);
%plot(model);

%% train
trainResults = train(TD3agent,env,trainOpt)

%% save agent

actor = getActor(TD3agent);
model = getModel(actor);
filename = "savedAgents/Simple2TD3_" + string(datetime('now','Format','MM_dd-HH_mm')) + "_ar" + string(floor(trainResults.AverageReward(end))) + "_l"+string(layerSize) + ".mat"
save(filename, "TD3agent", "trainResults")

%%

load('savedAgents/Simple_2_TD3_05_13-16_48_l31/Agent504.mat')
saved_agent = TD3agent;

% figure()
% hold on
% plot(trainResults.EpisodeReward)
% plot(trainResults.AverageReward)
% plot(trainResults.EpisodeQ0)
% grid()
% hold off

figure()
hold on
plot(savedAgentResult.EpisodeReward)
plot(savedAgentResult.AverageReward)
plot(savedAgentResult.EpisodeQ0)
hold off

[m, i] = max(trainResults.AverageReward)