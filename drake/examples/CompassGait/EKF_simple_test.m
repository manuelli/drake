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

%% Extract the nominal trajectory
xtraj_single_step = xtraj.traj{2};
xtraj_single_step = xtraj_single_step(2:5);

close all;


%% Set simulation parameters
plant = TimeSteppingCompassGaitPlant(gammaIn);
cgUtils = CompassGaitUtils();
t_start = 0.5;
t = t_start;
dt = 0.005;
x_full = xtraj.eval(t);
hybridMode = x_full(1);
x_local = x_full(2:end);
t_f = 1.5;

numTimesteps = ceil((t_f-t)/dt);
t_f = t + numTimesteps*dt;
x_final = xtraj.eval(t_f);
xGlobal = cgUtils.transformLocalStateToGlobalState(hybridMode, x_local);

t_current = t;

x_final_global = cgUtils.transformLocalStateToGlobalState(x_final(1), x_final(2:end));

%% Setup the utilities


runOptions = struct();
runOptions.useParticleFilter = true;
runOptions.useEKF = true;

options = struct();
options.initializationPositionNoiseStdDev = 0.01;
options.initializationVelocityNoiseStdDev = 0.05;

options.processNoiseStdDev_v = 0.02;
options.processNoiseStdDev_v = 0.1;


options.measurementNoiseIMUVar = dt*0.01;
options.measurementNoiseEncodersVar = dt*0.01;
options.numParticles = 100;

truthParticleOptions = struct();
truthParticleOptions.numParticles = 0;




particleFilter = CompassGaitParticleFilter(plant, options);
particleFilter.nominalTraj_ = xtraj_single_step;


ekf = CompassGaitExtendedKalmanFilter(plant, options);

% determines whether or not we add noise to the measurements
observationOptions = struct();
observationOptions.addNoise = false;




%% Test the particle filter functionality

if runOptions.useParticleFilter
  particleFilter.initializeFilter(hybridMode, xGlobal);
end

ekf.initializeFilter(hybridMode, xGlobal);

inputData = struct('xGlobal',xGlobal);
inputData.hybridMode = hybridMode;
particle = CompassGaitParticle(inputData);
trueParticle = CompassGaitParticle(inputData);

trueParticleArray = {};
particleSetArray = {};
particleSetArrayAfterImportanceResampling = {};
observationArray = {};
kalmanFilterParticleArray = {};
kalmanFilterParticleBarArray = {};
tArray = [];
xLocalArray = [];

hybridEventTimes = [];

uGlobal = 0;

particleFilterActive = false;
particleFilter.particleSet_ = {};

tic;
% profile on;
% do a bunch of forward simulations
for i=1:numTimesteps

  hybridEvent = false; % helper var

  % move the true particle
  % don't use uncertainty for now
  outputData = particleFilter.applyMotionModelSingleParticle(trueParticle,uGlobal,dt,struct('useUncertainty',false));
  
  
  if(isfield(outputData,'hybridEventTime'))
     hybridEventTimes(end+1) = t_current + outputData.hybridEventTime;
     hybridEvent = true;
  end
  
  trueParticleArray{end+1} = CompassGaitParticle.copy(trueParticle);

  % move the particles in the filter using the stochastic motion model
%   particleFilter.applyMotionModel(0,dt);


  % if needed, apply the reset map to the ekf
  if hybridEvent
    ekf.applyResetMap();
  end
  
  % apply the measurement update

  [y, yParticle] = particleFilter.generateObservation(trueParticle, dt, observationOptions);

  observationArray{end+1} = yParticle; % record the observations that we had

  if runOptions.useEKF
    % for now we will tell the EKF the correct hybrid mode
    % apply the motion model
    ekf.applyMotionModel(uGlobal, dt);
    ekf.applyMeasurementUpdate(trueParticle.hybridMode_, y);
    kalmanFilterParticleArray{end+1} = ekf.getKalmanFilterStateAsParticle();
    kalmanFilterParticleBarArray{end+1} = ekf.getKalmanFilterBarStateAsParticle();
  end


  if runOptions.useParticleFilter
    
    if (~particleFilterActive)
      kfParticle  = kalmanFilterParticleArray{end};
      xLocal = cgUtils.transformGlobalStateToLocalState(kfParticle.hybridMode_, kfParticle.x_);
      stanceAngle = xLocal(2);
      
      if (stanceAngle < 0)
        sigmaTest = 2*kfParticle.sigma_;
        returnData = particleFilter.initializeFilter(kfParticle.hybridMode_, kfParticle.x_, struct('sigmaGlobal', kfParticle.sigma_));
        if (returnData.hitHybridGuard)
           
          disp('activating particle filter')
          particleFilter.initializeFilter(kfParticle.hybridMode_, kfParticle.x_, struct('sigmaGlobal', kfParticle.sigma_));
          pfActivationMode = kfParticle.hybridMode_;
          particleFilterActive = true;
        end
      end
    end
    
    if (particleFilterActive)    
      % apply motion model
      particleFilter.applyMotionModel(0,dt);

      % store the particle set
      particleSetArray{end+1} = CompassGaitParticle.copyParticleSet(particleFilter.particleSet_);

      % HACK: add truth particles if necessary
  %     particleFilter.addTruthParticles(trueParticle, truthParticleOptions);

      % apply the measurement update
      particleFilter.applyMeasurementUpdate(y,dt);
      particleFilter.applyImportanceResampling();

    % store the particle set after importance resampling
      particleSetArrayAfterImportanceResampling{end+1} = CompassGaitParticle.copyParticleSet(particleFilter.particleSet_);
    else
      % otherwise store empty ones
      particleSetArray{end+1} = {};
      particleSetArrayAfterImportanceResampling{end+1} = {};
    end
    
    
    % deactivate PF when all in the next mode
    if (particleFilterActive & (trueParticle.hybridMode_ ~= pfActivationMode))
      if ((particleFilter.particleSet_{1}.hybridMode_ ~= pfActivationMode) && CompassGaitParticle.allParticlesInSameMode(particleFilter.particleSet_))
        disp('deactivating PF');
        particleFilterActive = false;
      end
%       kfParticle  = kalmanFilterParticleArray{end};
%       xLocal = cgUtils.transformGlobalStateToLocalState(kfParticle.hybridMode_, kfParticle.x_);
%       stanceAngle = xLocal(2);
%       if (stanceAngle > 0.155)
%         particleFilterActive = false;
%         disp('deactivating PF');
%       end
    end
    
    
  end

  t_current = t_current + dt;
  tArray(end+1) = t_current;
  xLocalArray(:,end+1) = cgUtils.transformGlobalStateToLocalState(trueParticle.hybridMode_, trueParticle.x_);
end
% profile viewer
toc;

%% Make Trajetory of true particle

simTraj = PPTrajectory(pchip(tArray, xLocalArray));

% visualize the trajectory
v = CompassGaitVisualizer(plant, simTraj.getOutputFrame);
v.playback(simTraj, struct('slider',true));


%% Interactive plotting
figCounter = 5;
fig = figure(figCounter);
figCounter = figCounter + 1;

plotData = struct();
plotData.ekf = ekf;
plotData.particleFilter = particleFilter;
plotData.trueParticleArray = trueParticleArray;
plotData.observationArray = observationArray;
plotData.times = tArray;

if runOptions.useEKF
  plotData.kalmanFilterParticleArray = kalmanFilterParticleArray;
end

if runOptions.useParticleFilter
%   plotData.particleSetArray = particleSetArray;
  plotData.particleSetArray = particleSetArrayAfterImportanceResampling;
end

plotParticles(plotData, fig)

% plotDataAfterResampling = plotData;
% plotDataAfterResampling.particleSetArray = particleSetArray;
% plotDataAfterResampling.plotWeights = true;
% fig = figure(figCounter);
% figCounter = figCounter + 1;

% plotParticles(plotDataAfterResampling, fig);

