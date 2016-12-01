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

close all;
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
options.initializationPositionNoiseStdDev = 0.02;
options.initializationVelocityNoiseStdDev = 0.05;
options.processNoiseStdDev_v = 0.1;


options.measurementNoiseIMUStdDev = 0.005;
options.measurementNoiseEncodersStdDev = 0.001;
options.numParticles = 100;


particleFilter = CompassGaitParticleFilter(plant, options);
particleFilter.nominalTraj_ = xtraj_single_step;

%% Test simulation with ode45 vs. ode4



%% Test the particle filter functionality
t_start = 0.3;
t = t_start;
x_full = xtraj.eval(t);
hybridMode = x_full(1);
x_local = x_full(2:end);
t_f = 1.5;
dt = 0.005;
numTimesteps = ceil((t_f-t)/dt);
t_f = t + numTimesteps*dt;
x_final = xtraj.eval(t_f);
xGlobal = cgUtils.transformLocalStateToGlobalState(hybridMode, x_local);

t_current = t;

x_final_global = cgUtils.transformLocalStateToGlobalState(x_final(1), x_final(2:end));

particleFilter.initializeFilter(hybridMode, xGlobal);

inputData = struct('xGlobal',xGlobal);
inputData.hybridMode = hybridMode;
particle = CompassGaitParticle(inputData);
trueParticle = CompassGaitParticle(inputData);

trueParticleArray = {};
particleSetArray = {};
particleSetArrayAfterImportanceResampling = {};
tArray = [];
xLocalArray = [];

hybridEventTimes = [];

uGlobal = 0;

tic;
profile on;
% do a bunch of forward simulations
for i=1:numTimesteps
  % move the true particle
  % don't use uncertainty for now
  outputData = particleFilter.applyMotionModelSingleParticle(trueParticle,uGlobal,dt,struct('useUncertainty',false));
  
  if(isfield(outputData,'hybridEventTime'))
     hybridEventTimes(end+1) = t_current + outputData.hybridEventTime;
  end
  trueParticleArray{end+1} = CompassGaitParticle.copy(trueParticle);

  % move the particles in the filter using the stochastic motion model
  particleFilter.applyMotionModel(0,dt);
  particleSetArray{end+1} = CompassGaitParticle.copyParticleSet(particleFilter.particleSet_);

  % perform measurement update
  y = particleFilter.generateObservation(trueParticle,dt);
  y = [trueParticle.x_.qL; trueParticle.x_.qR];
  particleFilter.applyMeasurementUpdate(y,dt);

  % do importance resampling
  particleFilter.applyImportanceResampling();
  particleSetArrayAfterImportanceResampling{end+1} = CompassGaitParticle.copyParticleSet(particleFilter.particleSet_);

  t_current = t_current + dt;
  tArray(end+1) = t_current;
  xLocalArray(:,end+1) = cgUtils.transformGlobalStateToLocalState(trueParticle.hybridMode_, trueParticle.x_);
end
profile viewer
toc;

%% Make Trajetory of true particle

simTraj = PPTrajectory(pchip(tArray, xLocalArray));

% visualize the trajectory
v = CompassGaitVisualizer(plant, simTraj.getOutputFrame);
v.playback(simTraj, struct('slider',true));

fig = figure();
hold on;
h = fnplt(simTraj,[1]);
set(h, 'Color','b', 'DisplayName','swing');
h = fnplt(simTraj,[2]);
set(h, 'Color','r', 'DisplayName', 'stance');
hold off;

%% Interactive plotting
figCounter = 5;
fig = figure(figCounter);
figCounter = figCounter + 1;

plotData = struct();
plotData.particleFilter = particleFilter;
plotData.particleSetArray = particleSetArray;
plotData.trueParticleArray = trueParticleArray;
plotData.times = tArray;

plotParticles(plotData, fig)

plotDataAfterResampling = plotData;
plotDataAfterResampling.particleSetArray = particleSetArrayAfterImportanceResampling;
plotDataAfterResampling.plotWeights = true;
fig = figure(figCounter);
figCounter = figCounter + 1;

plotParticles(plotDataAfterResampling, fig);

