%% Get trajectory of passive Compass Gait
% This is  in order to a pass a reasonable initial trajectory to the trajectory optimization
% First we simulate a passive compass gait on a slope
gammaIn = 3*pi/180;
r = CompassGaitPlant(gammaIn);

x0 = r.getInitialState;
T = 10;


%% simulation without feedback
[ytraj, xtraj] = simulate(r, [0, T], x0);
v = CompassGaitVisualizer(r);

%% Plot the nominal trajectory
xtraj_single_step = xtraj.traj{2};
xtraj_single_step = xtraj_single_step(2:5);
fig1 = figure(1);
clf(fig1);

fig2 = figure(2);
clf(fig2);
% fig1 = figure(1);
% hold on;
% fnplt(xtraj_single_step, [1,3]);
% fnplt(xtraj_single_step, [2,4]);
% hold off;
% 
% fig2 = figure(2);
% hold on;
% fnplt(xtraj_single_step, [1,3]);
% fnplt(xtraj_single_step, [2,4]);
% hold off;
% 
% fig2 = figure(2);
% hold on;
% fnplt(xtraj_single_step, [1,3]);
% fnplt(xtraj_single_step, [2,4]);
% hold off;

%% Setup the utilities
plant = TimeSteppingCompassGaitPlant(gammaIn);
cgUtils = CompassGaitUtils();

options = struct();
options.initializationPositionNoiseStdDev = 0;
options.initializationVelocityNoiseStdDev = 0;
options.processNoiseStdDev_v = 0.2;
options.numParticles = 10;


particleFilter = CompassGaitParticleFilter(plant, options);
particleFilter.nominalTraj_ = xtraj_single_step;

%% Test simulation with ode45 vs. ode4



%% Test the particle filter functionality
t = 0.3;
x_full = xtraj.eval(t);
hybridMode = x_full(1);
x_local = x_full(2:end);
t_f = 0.5;
dt = 0.01;
numTimesteps = ceil((t_f-t)/dt);
t_f = t + numTimesteps*dt;
x_final = xtraj.eval(t_f);
xGlobal = cgUtils.transformLocalStateToGlobalState(hybridMode, x_local);

t_current = t;

x_final_global = cgUtils.transformLocalStateToGlobalState(x_final(1), x_final(2:end));

particleFilter.initializeFilter(hybridMode, xGlobal);

% close all;
% fig1 = figure(1);
particleFilter.plotParticleSet(particleFilter.particleSet_, fig1);
hold on;
scatter(xGlobal.qL, xGlobal.vL, 'g', 'filled');
title('t = 0');
hold off;

tic;
% profile on;
% do a bunch of forward simulations
for i=1:numTimesteps
  particleFilter.applyMotionModel(0,dt);
  t_current = t_current + dt;
end
% profile viewer
toc;

% fig2 = figure(2);
hold on;
particleFilter.plotParticleSet(particleFilter.particleSet_, fig2);
scatter(x_final_global.qL, x_final_global.vL, 'g', 'filled');
title('t = t_f');
hold off;


% tic;
% % do a bunch of forward simulations
% for i=1:numTimesteps
%   particleFilter.applyMotionModel(0,dt);
%   t_current = t_current + dt;
% end
% toc
%   








%% Plot the particle




% xGlobal = cgUtils.transformLocalStateToGlobalState(hybridMode, x_local)
% inputData = struct('xGlobal',xGlobal);
% inputData.hybridMode = hybridMode;
% particle = CompassGaitParticle(inputData);
% 
% 
% % Try to simulate the particle forward with 0 control input for 0.2 seconds
% dt = 0.7;
% t_f = t + dt;
% uGlobal = 0;
% particleFilter.applyMotionModelSingleParticle(particle, uGlobal, dt);
% 
% x_final_global = particle.x_
% 
% x_final_traj = xtraj.eval(t_f);
% x_final_traj_global = cgUtils.transformLocalStateToGlobalState(x_final_traj(1), x_final_traj(2:end))