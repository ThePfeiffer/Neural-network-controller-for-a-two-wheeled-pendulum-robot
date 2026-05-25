%% init
close all
clear
clc
%%
seed = 5;
rng(seed, 'twister');
load('TunedPIDworkspace.mat')
open('robot_locomotion_PIDvsNN.slx')
%%
load("savedAgents/Simple_TD3_05_11-11_50_l31 - Final/Agent1883.mat")
TD3agent = saved_agent;
Net = getModel(getActor(TD3agent));

%%
useNN = false;
%%
useNN = true;
%% Random mission generator
% range limited gaussian
function x = normRc(mu, sigma, MIN, MAX)
% x = normrnd(mu, sigma) % normally distrubuted number
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
tStart = 0.2;
sLength = 5;
physicalTestMission_1 = [   0, 0, 0;
                            tStart, 0.2, 0;
                            sLength+tStart, 0.4, 0;
                            2*sLength+tStart, 0, 0;
                            3*sLength+tStart, -0.4, 0;
                            4*sLength+tStart, 0.5, 0;
                            5*sLength+tStart, 0.1, 0;
                            6*sLength+tStart, 0, 0;
                            7*sLength+tStart, 0, 0];


physicalTestMission_2 = [   0, 0, 0;
                            tStart, 0.2, 0;
                            2, 0.4, 0;
                            3, 0, 0;
                            4.5, -0.4, 0;
                            7.5, 0.5, 0;
                            9.5, 0.1, 0;
                            10.5, 0, 0;
                            11.5, 0, 0];

%%
% transfer functions for optimal reference.
% velocity:
s = tf('s');
tn = 0.35; %0.13 %pid 0.35
wn = 1/tn;
zeta = 0.9; %pid 0.9
velFunc = wn^2/(wn^2 + 2*zeta*wn*s + s^2); % 2. order

stepinfo(velFunc)

% tilt
tn = 0.37;  %pid 0.37
wn = 1/tn; 
zeta = 0.7; % %pid 0.7
adjust = 0.6; %pid 0.6
tiltFunc = -s*adjust*wn^2/(wn^2 + 2*zeta*wn*s + s^2); % 2. order

tiltVelFunc = tiltFunc*s;
stepinfo(tiltFunc)

% figure()
% step(velFunc)
% hold on;
% step(tiltFunc)
% % step(tiltVelFunc)
% hold off
% xlim([0, 10])
% ylim([-0.9, 1.1])

%% Simulate
useNN = false;
velStep = physicalTestMission_1
tEnd = velStep(end/3)

%% generate test missions
nMis = 30;

Missions = [];
Ends = [];
for i=1:nMis
    [velStep tEnd] = VelMissionGen();
    Missions = [Missions; velStep];
    Ends(i) = tEnd;
end
nSeg = size(velStep,1);
filename = 'Missions_PIDvsNN_'+string(datetime('now','Format','yyyy_MM_dd-HH_mm'))+'_n'+string(nMis)+'_s'+string(seed)+'.mat'
save(filename, 'Missions', 'Ends','nSeg');


%% Simulate models for test
load('Missions_PIDvsNN_2026_05_12-13_59_n30_s5.mat')
nMis = length(Ends);

% start with PID
useNN = false;
simIn(1:nMis) = Simulink.SimulationInput('robot_locomotion_PIDvsNN_test');

for i = 1:nMis
    velStep = Missions((nSeg*(i-1)+1:nSeg*i), :);
    tEnd = Ends(i);

    simIn(i) = simIn(i).setVariable('velStep', velStep);
    simIn(i) = simIn(i).setVariable('tEnd', tEnd);
end

%simulate
simOut = parsim(simIn, 'UseFastRestart', 'on', "TransferBaseWorkspaceVariables", 'on');

filename = 'PID_test_data_'+string(datetime('now','Format','yyyy_MM_dd-HH_mm'))+'_n'+string(nMis)+'.mat'
save(filename, 'simOut', 'nMis');

%%
% Then NN
useNN = true;
simIn(1:nMis) = Simulink.SimulationInput('robot_locomotion_PIDvsNN_test');

for i = 1:nMis
    velStep = Missions((nSeg*(i-1)+1:nSeg*i), :);
    tEnd = Ends(i);

    simIn(i) = simIn(i).setVariable('velStep', velStep);
    simIn(i) = simIn(i).setVariable('tEnd', tEnd);
end

%simulate
simOut = parsim(simIn, 'UseFastRestart', 'on', "TransferBaseWorkspaceVariables", 'on');


filename = 'NN_test_data_'+string(datetime('now','Format','yyyy_MM_dd-HH_mm'))+'_n'+string(nMis)+'.mat';
save(filename, 'simOut', 'nMis');

%%
% plot simulation results
load("PID_test_data_2026_05_12-14_01_n30.mat")

rewardsPID = [];
for i = 1:nMis
    rewardsPID = [rewardsPID, sum(simOut(i).OutReward.data)];
end
avgRewardPID = sum(rewardsPID)/nMis


load("NN_test_data_2026_05_12-14_02_n30.mat")

rewardsNN = [];
for i = 1:nMis
    rewardsNN = [rewardsNN, sum(simOut(i).OutReward.data)];
end

avgRewardNN = sum(rewardsNN)/nMis


figure(Theme='light')
plot(rewardsPID, 'LineWidth',2)
hold on
plot(rewardsNN,  'LineWidth',2)
plot(rewardsPID-rewardsNN,  'LineWidth',2)
% plot(zeros(nMis),  'LineWidth',1, 'Color', [150/255, 150/255, 150/255])
hold off
legend(["PID", "NN", "Difference (PID - NN)"])
title("Reward per test mission")
xlabel("Mission number")
xlim([1,nMis])
ylabel("Reward")
grid()

%%
% laod mission 10
load('Missions_PIDvsNN_2026_05_12-13_59_n30_s5.mat')
nMis = length(Ends);

i = 10;
velStep = Missions((nSeg*(i-1)+1:nSeg*i), :);
tEnd = Ends(i);

useNN = true;

%% Plot log from single Robot

%  1    time 35.3501 sec, from Viking (94)
%  2  3  4 Gyro x,y,z [deg/s]: -2.38574 35.5381 -0.564307
%  5  6 Motor voltage [V] left, right: 10.00 10.00
%  7 10 Wheel velocity [m/s] left, right: -0.0026 0.0021 delay 0.0855 0.0991 (sec)
% 11 12 13 14 Pose x,y,h,tilt [m,m,rad,rad]: -0.102196 -0.396848 -2.39013 0.225189
% 15 24 ctrl bal vel, ref=0, m=0.00213701, m2=-0.210283, uf=0, r2=0, ep=-0.420566,up=-0.086464, ui=0, u1=-0.086464, u=-0.086464

aa = load("Robot_tests/_NN_Viking_test_1_final.txt");
aa = cast(aa, "single"); 
time = aa(:, 1);

%Gyro in rad/s
aa(:, 2) = aa(:, 2)*pi/180; 
aa(:, 3) = aa(:, 3)*pi/180; 
aa(:, 4) = aa(:, 4)*pi/180;

%velocity
aa(:,16) = (aa(:,7)+aa(:,8))/2;

toplot = [3];
legends = ["time (s)", "Gyro x (rad/s)", "Gyro y (10 rad/s)", "Gyro z (rad/s)", "Left (V)", "Right (V)", "Left (m/s)", "Right (m/s)","delay", "delay", "x (m)", "y (m)", "h (rad)", "Tilt (rad)", "Mission Vel Ref (m/s)", "Velocity (m/s)"];
leg = [];
plotindex = 1;

figure(Theme="light")
hold on
for i=1:16
    if ~(plotindex > size(toplot, 2))
        if (i == toplot(plotindex))
            plot(time, aa(:, i), '-', 'LineWidth',1.3, 'color', [196/256, 95/256, 38/256]) % Left Wheel vel m/s
            plotindex = plotindex+1;
            leg = [leg, legends(i)];
        end
    end
end 

legend(leg)
xlim([0, time(end)])
ylim([-7, 3])
title("Final Test Mission - Viking - Tilt velocity")
xlabel("Time [s]")
ylabel("Tilt velocity [rad/s]")
grid()



%% Plot log from multiple Robots
%  1    time 35.3501 sec, from Viking (94)
%  2  3  4 Gyro x,y,z [deg/s]: -2.38574 35.5381 -0.564307
%  5  6 Motor voltage [V] left, right: 10.00 10.00
%  7 10 Wheel velocity [m/s] left, right: -0.0026 0.0021 delay 0.0855 0.0991 (sec)
% 11 12 13 14 Pose x,y,h,tilt [m,m,rad,rad]: -0.102196 -0.396848 -2.39013 0.225189
% 15 24 ctrl bal vel, ref=0, m=0.00213701, m2=-0.210283, uf=0, r2=0, ep=-0.420566,up=-0.086464, ui=0, u1=-0.086464, u=-0.086464

aa = load("Robot_tests/_NN_Viking_test_1_final.txt");
aa = cast(aa, "single"); 
time = aa(:, 1);

%Gyro in rad/s
aa(:, 2) = aa(:, 2)*pi/180; 
aa(:, 3) = aa(:, 3)*pi/180; 
aa(:, 4) = aa(:, 4)*pi/180;

%velocity
aa(:,16) = (aa(:,7)+aa(:,8))/2;

% voltage
%aa(:, 5:6) = aa(:, 5:6)/10;

% d = diff(aa(:,14))/(time(end)/size(time,1)); % differentiate the tilt to
% get tiltvel
% aa(:,6) = [0; d(:, 1:end)];


toplot = [3];
legends = ["time (s)", "Gyro x (rad/s)", "Gyro y (10 rad/s)", "Gyro z (rad/s)", "Left (V)", "Right (V)", "Left (m/s)", "Right (m/s)","delay", "delay", "x (m)", "y (m)", "h (rad)", "Tilt (rad)", "Mission Vel Ref (m/s)", "Velocity (m/s)"];
leg = [];
plotindex = 1;

figure(Theme="light")
hold on
for i=1:16
    if ~(plotindex > size(toplot, 2))
        if (i == toplot(plotindex))
            plot(time, aa(:, i), '-', 'LineWidth',1.3) % Left Wheel vel m/s
            plotindex = plotindex+1;
            leg = [leg, legends(i)];
        end
    end
end 

aa = load("Robot_tests/_NN_Falk_test_1_final.txt");
aa = cast(aa, "single"); 
time = aa(:, 1);

%Gyro in rad/s
aa(:, 2) = aa(:, 2)*pi/180; 
aa(:, 3) = aa(:, 3)*pi/180; 
aa(:, 4) = aa(:, 4)*pi/180;

%velocity
aa(:,16) = (aa(:,7)+aa(:,8))/2;

% voltage
%aa(:, 5:6) = aa(:, 5:6)/10;

% d = diff(aa(:,14))/(time(end)/size(time,1)); % differentiate the tilt to
% get tiltvel
% aa(:,6) = [0; d(:, 1:end)];


toplot = [3];
legends = ["time (s)", "Gyro x (rad/s)", "Gyro y (10 rad/s)", "Gyro z (rad/s)", "Left (V)", "Right (V)", "Left (m/s)", "Right (m/s)","delay", "delay", "x (m)", "y (m)", "h (rad)", "Tilt (rad)", "Mission Vel Ref (m/s)", "Velocity (m/s)"];
plotindex = 1;

for i=1:16
    if ~(plotindex > size(toplot, 2))
        if (i == toplot(plotindex))
            plot(time, aa(:, i), '-', 'LineWidth',1.3) % Left Wheel vel m/s
            plotindex = plotindex+1;
            leg = [leg, legends(i)];
        end
    end
end 
grid()
legend(leg)
legend('NN Viking', 'NN Falk')
xlim([0, time(end)])
% ylim([-11, 11])
title("Final Test Mission - Tilt velocity of controllers")
xlabel("Time [s]")
ylabel("Tilt velocity [rad/s]")



%% frequency spectrum of noise
aa = load("Robot_tests/_PID_Viking_test_1_final.txt");
data = aa(:, 3);
N = size(data, 1);
t = aa(:, 1);
fs = 100;
f = fs*[0:N-1]/N;

Y = 20*log10(abs(fft(data)));
figure()
plot(f, Y)

