%% Get trajectory of passive Compass Gait
% This is  in order to a pass a reasonable initial trajectory to the trajectory optimization
% First we simulate a passive compass gait on a slope
close all;
gammaIn = 3*pi/180;
r = CompassGaitPlant(gammaIn);

x0 = r.getInitialState;
T = 10;


%% simulation without feedback
% [ytraj, xtraj] = simulate(r, [0, T], x0);
% v = CompassGaitVisualizer(r);


%% Optimize a trajectory for flat ground
% gammaOpt = 0*pi/180;
gammaSlopeInteger = 3;
gammaOpt = gammaSlopeInteger*pi/180; % do it on a zero slope
p = CompassGaitPlant(gammaOpt);
options = struct();
options.numKnotPoints = 40;
options.plant = p;
options.u_const_across_transitions = false;
options.stanceLegSweepAngleLowerBound = 0.25;
options.useDeltaUCost = false;
options.deltaUCostWeight = 3.0;

% segmentIdx = xtraj.traj{8};
% xtraj_single_step = xtraj.traj{8};
% xtraj_single_step_no_mode = xtraj_single_step(2:5);
% [p, utraj_opt, xtraj_opt] = SingleStepOptimization(xtraj_single_step_no_mode, options);

% load trajectory from file.
% this trajectory has a nominal slope of gamma = 3, but extends to the gamma = 4 reset
data = load('trajectory_data.mat');
xtraj_opt = data.returnData.xtraj;
utraj_opt = data.returnData.utraj;



gammaVals = [2;3;4];
plotNominalTrajectory(xtraj_opt, gammaVals);

%% Extract the nominal trajectory
xtraj_single_step = xtraj_opt;

%% Setup HZD Controller
% standard, no uncertainty crap here
options = struct();
options.Kp = 50;
options.dampingRatio = 1.0;
options.useLQR = true; % derives PD gains from an LQR problem
options.usePlanStanceLegVelocity = false;
hzdController = HZDController(p, options);
hzdController = hzdController.setNominalTrajectory(xtraj_opt, utraj_opt);
hzdController.plotPhaseTrajectories();

%% Full state LQR
Q = 1*diag([2,2,1,1]);
R = 0.1;
cgLQR = CompassGaitLQR(p, xtraj_opt, utraj_opt);
[tGrid, S_traj, K_traj] = cgLQR.computeLQRValueFunction(Q,R);


fig = figure(55);
clf(fig);
numPlots = 3;
subplot(numPlots,1,1);
hold on;
plot(tGrid, K_traj(:,1), 'b', 'DisplayName','swing');
plot(tGrid, K_traj(:,2), 'r', 'DisplayName','stance');
legend('show');
title('position gains');
hold off;

subplot(numPlots,1,2);
hold on;
plot(tGrid, K_traj(:,3), 'b', 'DisplayName','swing');
plot(tGrid, K_traj(:,4), 'r', 'DisplayName','stance');
title('velocity gains');
legend('show');
hold off;

subplot(numPlots,1,3);
hold on;
plot(tGrid, S_traj(:,1,1), 'b', 'DisplayName','swing');
plot(tGrid, S_traj(:,2,2), 'r', 'DisplayName','stance');
title('Value function');
legend('show');
hold off;


%% PD Controller

% this is just another PD controller
pdControllerOptions = struct();
pdControllerOptions.Kp = 50;
pdController = CompassGaitPDController(p, xtraj_opt, utraj_opt, pdControllerOptions);


%% Set simulation parameters
plant = TimeSteppingCompassGaitPlant(p.gamma);

plant_PF = TimeSteppingCompassGaitPlant(p.gamma);

cgUtils = CompassGaitUtils();
t_start = 0.3;
t = t_start;
dt = 0.0025;
x_local_orig = xtraj_opt.eval(t);

delta_x = [0;0;-0.0;0];
x_local = x_local_orig + delta_x;


hybridMode = 1;
t_f = 3;

numTimesteps = ceil((t_f-t)/dt);
t_f = t + numTimesteps*dt;
% x_final = xtraj.eval(t_f);
xGlobal = cgUtils.transformLocalStateToGlobalState(hybridMode, x_local);

t_current = t;

% x_final_global = cgUtils.transformLocalStateToGlobalState(x_final(1), x_final(2:end));

%% Setup the options for EKF and PF

runOptions = struct();
runOptions.useParticleFilter = true;
runOptions.useEKF = false;
runOptions.useObserver = true;



% runOptions.gammaVals = gammaSlopeInteger*pi/180;
runOptions.gammaVals = [4;3;3;3]*pi/180;


% these options apply to the EKF and also to the Particle Filter
options = struct();
options.initializationPositionNoiseStdDev = 0.01;
options.initializationVelocityNoiseStdDev = 0.01;

options.processNoiseStdDev_q = 1e-5; % was 0.01
options.processNoiseStdDev_v = 1e-5; % was 0.01


options.measurementNoiseIMUVar = dt*0.00;
options.measurementNoiseEncodersVar = 2*dt*1e-4;
options.numParticles = 100;

truthParticleOptions = struct();
truthParticleOptions.numParticles = 0;


%% Controller Options
controllerOptions = struct();
% set the type of the controller
% options are
% - hzd
% - lqr
% - pd
controllerOptions.controlStrategy = 'hzd';

switch controllerOptions.controlStrategy
  case 'hzd'
    controller = hzdController;
  case 'pd'
    controller = pdController;
end

% options include
% - normal
% - deadzone
% - robust
% - robust_standard_control
controllerOptions.type = 'normal'; % just standard controller
controllerOptions.robustControlDurationAfterReset = 0.16;
% what state to use in the controller
% options include
% - true
% - ekf
% - observer
controllerOptions.controlState = 'true'; 
controllerOptions.estimatorIdx = 1; % which estimator to use, early, normal, late etc.
% controllerOptions.usePlanStanceLegVelocity = true;

controllerOptions.useHackyStuff = false;
controllerOptions.hackThetaLowerBound = -0.075;
controllerOptions.hackThetaUpperBound = 0.075;


ekfOptions = struct();

% type of EKF resets
% - 'early'
% - 'mid'
% - 'late'
% - 'natural'
ekfOptions.resetType = 'true';
ekfOptions.dt = dt;

%% Setup Luenberger Observer
cgObserverOptions = struct();
cgObserverOptions.epsilon = 0.05; % this controls the observer gain basically
cgObserver = CompassGaitObserver(plant, cgObserverOptions);
cgObserver.initialize(hybridMode, xGlobal);

%% Setup Particle Filter
pfOptions = options;
pfOptions.epsilonObserverGain = cgObserverOptions.epsilon;
pfOptions.sigmaGlobalInit = 1e-5*eye(4);
pfOptions.sigmaGlobalTest = 1e-4*eye(4); % this tests for collision against the guard
pfOptions.numParticlesForEachTerrainHeight = [25;25;25];

pfOptions.deactivationTimeAfterLastReset = 0.1;
% pfOptions.gammaVals = gammaSlopeInteger*[1;1;1]*pi/180;
pfOptions.gammaVals = [gammaSlopeInteger-1, gammaSlopeInteger, gammaSlopeInteger+1]*pi/180;


pfOptions.resetObserverOnParticleFilterDeActivation = true;
pfOptions.applyObserverMeasurementUpdate = true;



particleFilter = CompassGaitParticleFilter(plant_PF, pfOptions);
particleFilter.nominalTraj_ = xtraj_single_step;

%% Setup EKF
% standard one
ekf = CompassGaitExtendedKalmanFilter(plant, options);

% this ekf has a built in reset map, i.e. it uses the reset map of the plant
% ekf_normal_reset = CompassGaitExtendedKalmanFilter(plant, options);

% an array of EKF's that get plotted
ekfArray = {};
ekfArray{1} = CompassGaitExtendedKalmanFilter(plant, options);

% this will be an 'early reset EKF', i.e. we give it the wrong terrain model
gamma_early_reset = (gammaSlopeInteger - 1)*pi/180;
plant_early_reset = TimeSteppingCompassGaitPlant(gamma_early_reset);
ekfArray{2} = CompassGaitExtendedKalmanFilter(plant_early_reset,options);


% late reset plant
gamma_late_reset = (gammaSlopeInteger + 1)*pi/180;
plant_late_reset = TimeSteppingCompassGaitPlant(gamma_late_reset);
ekfArray{3} = CompassGaitExtendedKalmanFilter(plant_late_reset,options);

num_ekfs = length(ekfArray);

observerArray = {};
observerArray{1} = CompassGaitObserver(plant, cgObserverOptions);
observerArray{1}.name_ = 'normal reset observer';

observerArray{2} = CompassGaitObserver(plant_early_reset, cgObserverOptions);
observerArray{2}.name_ = 'early reset observer';

observerArray{3} = CompassGaitObserver(plant_late_reset, cgObserverOptions);
observerArray{3}.name_ = 'late reset observer';


num_observers = length(observerArray);


% determines whether or not we add noise to the measurements
observationOptions = struct();
observationOptions.addNoise = false;
observationOptions.bias = [0;0]; % a bias to add to the measurements, see how this affects performance





%% Test the particle filter functionality

if runOptions.useParticleFilter
  particleFilter.initializeFilter(hybridMode, xGlobal);
end

ekf.initializeFilter(hybridMode, xGlobal);

for i=1:num_ekfs
  ekfArray{i}.initializeFilter(hybridMode, xGlobal);
end


inputData = struct('xGlobal',xGlobal);
inputData.hybridMode = hybridMode;
particle = CompassGaitParticle(inputData);
trueParticle = CompassGaitParticle(inputData);

% set the initial ground slope if so desired
if ~isempty(runOptions.gammaVals)
  trueParticle.gamma_ = runOptions.gammaVals(1);
  runOptions.gammaVals = runOptions.gammaVals(2:end);
end

trueParticleArray = {};
particleSetArray = {};
particleSetArrayAfterImportanceResampling = {};
observationArray = {};
kalmanFilterParticleArray = {};
kalmanFilterParticleArray_normal_reset = {};
observerParticleArray = {};
kalmanFilterParticleBarArray = {};
tArray = [];
plan_time_array = [];
stanceAngleAfterReset = [];
planTimeAfterImpactArray = [];
uArray = [];
xLocalArray = [];
controlDataArray = {};

hybridEventTimes = [];
ekfResetTimes = [];


uGlobal = 0;
simulatorInitialized = false;

particleFilterActive = false;
particleFilter.particleSet_ = {};

kalmanFilterBankParticleArray = {};
ekfResetTimesArray = {};

observerBankParticleArray = {};
for i=1:num_ekfs
  kalmanFilterBankParticleArray{end+1} = {};
  ekfResetTimesArray{end+1} = [];

end

observerBankParticleArray = {};
observerResetTimesArray = {};
for i=1:num_observers
  observerBankParticleArray{end+1} = {};
  observerArray{i}.initialize(hybridMode, xGlobal);
  observerResetTimesArray{end+1} = [];
end

% just stores the 3 particles corresponding to the observers
observerParticleSetArray = {};



% hack for generating observation for the first tick.
% apply the measurement update
[y, yParticle] = particleFilter.generateObservation(trueParticle, dt, observationOptions);

observationArray{end+1} = yParticle; % record the observations that we had

t_plan = t_current; % just a placeholder for now . . . 

% helper variable for when to reset EKF
resetEKF = false;
tic;
% profile on;
% do a bunch of forward simulations
for i=1:numTimesteps

  hybridEvent = false; % helper var

  % store the current particle
  tArray(end+1) = t_current;
  trueParticleArray{end+1} = CompassGaitParticle.copy(trueParticle);
  xLocalArray(:,end+1) = cgUtils.transformGlobalStateToLocalState(trueParticle.hybridMode_, trueParticle.x_);

  % generate an observation
  [y, yParticle] = particleFilter.generateObservation(trueParticle, dt, observationOptions);

  % record the observations that we had
  observationArray{end+1} = yParticle; 


  %% Apply measurement update to all the estimators
  if runOptions.useEKF
    % for now we will tell the EKF the correct hybrid mode
    ekf.applyMeasurementUpdate(y); % note y is in global coords
    kalmanFilterParticleArray{end+1} = ekf.getKalmanFilterStateAsParticle();

    for i=1:num_ekfs
      ekfArray{i}.applyMeasurementUpdate(y);
      kalmanFilterBankParticleArray{i}{end+1} = ekfArray{i}.getKalmanFilterStateAsParticle();
    end

    % ekf_normal_reset.applyMeasurementUpdate(y);
    % kalmanFilterParticleArray_normal_reset{end+1} = ekf_normal_reset.getKalmanFilterStateAsParticle();
  end

  if runOptions.useObserver
    cgObserver.applyMeasurementUpdate(y, dt);
    observerParticleArray{end+1} = cgObserver.getObserverStateAsParticle();

    for i=1:num_observers
      observerArray{i}.applyMeasurementUpdate(y, dt);
      observerBankParticleArray{i}{end+1} = observerArray{i}.getObserverStateAsParticle();
    end
  end

  if runOptions.useParticleFilter
    if particleFilterActive

      % apply the observer update if specified.
      if pfOptions.applyObserverMeasurementUpdate
        particleFilter.applyObserverMeasurementUpdate(y, dt);
      end

      % apply the measurement update
      particleFilter.applyMeasurementUpdate(y,dt);
      particleFilter.applyImportanceResampling();
      particleSetArray{end+1} = CompassGaitParticle.copyParticleSet(particleFilter.particleSet_);
    else
      particleSetArray{end+1} = {};
    end
  end

  %% Compute Control Input
  % determine the state off which we will control. Can be the true state
  % or also can be an estimated state, like and ekf or particle filter
  if strcmp(controllerOptions.controlState, 'true')
    controlStateParticle = trueParticle;
    if (isempty(hybridEventTimes))
      t_plan = t_current;
    else
      t_plan = (t_current - hybridEventTimes(end))+planTimeAfterImpactArray(end);
    end
  elseif strcmp(controllerOptions.controlState, 'ekf')
    controlStateParticle = ekf.getKalmanFilterStateAsParticle();
    if (isempty(ekfResetTimes))
      t_plan = t_current;
    else
      t_plan = t_current - ekfResetTimes(end);
    end
  elseif strcmp(controllerOptions.controlState, 'observer')

    controlStateParticle = observerArray{controllerOptions.estimatorIdx}.getObserverStateAsParticle();
  end

  % set the control state particle
    

  % Specify which type of controller to use.
  % options
  % - hzd
  % - lqr
  % - pd
  switch controllerOptions.controlStrategy
    case 'hzd'
      [uGlobal, controlData] = hzdController.getControlInputFromGlobalState(t_current, controlStateParticle.hybridMode_, controlStateParticle.x_, controllerOptions);
    case 'lqr'

      if (isempty(hybridEventTimes))
        t_plan = t_current;
      else
        t_plan = t_current - hybridEventTimes(end);
      end
      [uGlobal, controlData] = cgLQR.getControlInputFromGlobalState(t_plan, controlStateParticle.hybridMode_, controlStateParticle.x_);
    case 'pd'
      [uGlobal, controlData] = pdController.getControlInputFromGlobalState(controlStateParticle.hybridMode_, controlStateParticle.x_);      
    otherwise
      error('controller type must be one of lqr, pd or hzd');
  end

  % Whether to apply the deadzone or robust controller things
  % if you have controllerOptions.type set to 'normal' it will be do nothing
  % if (particleFilterActive)
  %   switch controllerOptions.type
  %     case 'deadzone'
  %       uGlobal = 0;
  %     case 'robust'
  %       uGlobal = hzdController.computeRobustControlFromParticleSet(particleFilter.particleSet_, dt);
  %   end
  % end


  % allEstimatorsInSameMode = EstimatorUtils.allEstimatorsInSameMode(observerArray);
  % robustControlAfterReset = (t_current - observerArray{3}.lastResetTime_) < controllerOptions.robustControlDurationAfterReset;
  % observerParticleSet = EstimatorUtils.particleSetFromObserverArray(observerArray);
  % if (~allEstimatorsInSameMode || robustControlAfterReset)
  %   switch controllerOptions.type
  %     case 'deadzone'
  %       uGlobal = 0;
  %     case 'robust'
  %       uGlobal = hzdController.computeRobustControlFromParticleSet(observerParticleSet, dt);
  %   end
  % end

  if particleFilterActive
    switch controllerOptions.type
      case 'deadzone'
        uGlobal = 0;
      case 'robust'
        uGlobal = controller.computeRobustControlFromParticleSet(particleFilter.particleSet_, dt);

      case 'robust_standard_control'
        particleSetAvgData = CompassGaitParticle.getAvgParticleInMostLikelyMode(particleFilter.particleSet_);
        controlStateParticle = particleSetAvgData.avgParticleInMostLikelyMode;

        [uGlobal, controlData] = hzdController.getControlInputFromGlobalState(t_current, controlStateParticle.hybridMode_, controlStateParticle.x_, controllerOptions);
    end
  end

  % observerParticleSetArray{end+1} = observerParticleSet;

  % control input has been decided at this point.
  % record it
  uArray(end+1) = uGlobal;

  %% Apply motion model to true particle and estimators

  % move the true particle forwards using the motion model
  % record the hybrid event if one ocurred
  outputData = particleFilter.applyMotionModelSingleParticle(trueParticle,uGlobal,dt,struct('useUncertainty',false));
  
  % record any hybrid events that may have occurred
  if(isfield(outputData,'hybridEventTime'))
     hybridEventTimes(end+1) = t_current + outputData.hybridEventTime;
     hybridEvent = true;

     % change the ground slope
     if ~isempty(runOptions.gammaVals)
      disp('changing the ground slope')
      trueParticle.gamma_ = runOptions.gammaVals(1);
      runOptions.gammaVals = runOptions.gammaVals(2:end);
     end
    

     xLocal = cgUtils.transformGlobalStateToLocalState(trueParticle.hybridMode_, trueParticle.x_);

     stanceAngleAfterReset = xLocal(2);
     planTimeAfterImpactArray(end+1) = cgLQR.ttrajPhase.eval(stanceAngleAfterReset);
  end

  % record the resetting stuff for the EKF is using the 'true' EKF reset type
  % if needed, apply the reset map to the ekf
  if (hybridEvent && strcmp(ekfOptions.resetType, 'true'))
    resetEKF = true;
  end


  % decide whether or not to activate PF
  % depends on whether we will hit the hybrid guard
  % if runOptions.useParticleFilter
    
  %   if (~particleFilterActive)
  %     % kfParticle  = kalmanFilterParticleArray{end};
  %     seedParticle = observerArray{i}.getObserverStateAsParticle();
  %     xLocal = cgUtils.transformGlobalStateToLocalState(seedParticle.hybridMode_, seedParticle.x_);
  %     stanceAngle = xLocal(2);
      
  %     if (stanceAngle < 0)
  %       sigmaTest = particleFilterOptions;
  %       returnData = particleFilter.initializeFilter(kfParticle.hybridMode_, kfParticle.x_, struct('sigmaGlobal', kfParticle.sigma_));
  %       if (returnData.hitHybridGuard)
           
  %         disp('activating particle filter')
  %         particleFilter.initializeFilter(kfParticle.hybridMode_, kfParticle.x_, struct('sigmaGlobal', kfParticle.sigma_));
  %         pfActivationMode = kfParticle.hybridMode_;
  %         particleFilterActive = true;

  %         if strcmp(ekfOptions.resetType, 'early')
  %           resetEKF = true;
  %         end
  %       end
  %     end
  %   end
  % end

  % decide whether we should de-activate particle filter
  % if (runOptions.useParticleFilter && particleFilterActive)
  %   % all particles must be in the same mode, and that mode must be different than the 
  %   % mode in which the PF was initialized
  %   if CompassGaitParticle.allParticlesInSameMode(particleFilter.particleSet_)
  %     currentMode = particleFilter.particleSet_{1}.hybridMode_;
  %     if currentMode ~= pfActivationMode
  %       particleFilterActive = false;
  %       disp('de-activating particle filter')

  %       % trigger the 'late' EKF reset
  %       if strcmp(ekfOptions.resetType, 'late')
  %         resetEKF = true;
  %       end
  %     end
  %   end
  % end

  if runOptions.useParticleFilter
    
    if (~particleFilterActive)
      % kfParticle  = kalmanFilterParticleArray{end};
      seedParticle = observerArray{i}.getObserverStateAsParticle();
      xLocal = cgUtils.transformGlobalStateToLocalState(seedParticle.hybridMode_, seedParticle.x_);
      stanceAngle = xLocal(2);
      
      if (stanceAngle < 0)
        returnData = particleFilter.initializeFilterUncertainTerrainHeight(seedParticle, pfOptions.sigmaGlobalTest, pfOptions);
        if (returnData.hitHybridGuard)
           
          disp('activating particle filter')
          particleFilter.initializeFilterUncertainTerrainHeight(seedParticle, pfOptions.sigmaGlobalInit, pfOptions);
          particleFilterActive = true;
          hybridModeWhenParticleFilterActivated = seedParticle.hybridMode_;
        end
      end
    end
  end

  % decide whether we should de-activate particle filter
  if (runOptions.useParticleFilter && particleFilterActive)
    % the pf should have switched mode and also be beyond a given reset time
    if ( ((t_current - observerArray{3}.lastResetTime_) > pfOptions.deactivationTimeAfterLastReset) && (observerArray{3}.lastResetTime_ > 0) && (observerArray{3}.hybridMode_ ~= hybridModeWhenParticleFilterActivated))
      particleFilterActive = false;
      disp('de-activating particle filter');

      if pfOptions.resetObserverOnParticleFilterDeActivation
        d = CompassGaitParticle.getAvgParticleInMostLikelyMode(particleFilter.particleSet_);
        observerResetParticle = d.avgParticleInMostLikelyMode();
        observerArray{1}.hybridMode_ = observerResetParticle.hybridMode_;
        xLocal = cgUtils.transformGlobalStateToLocalState(observerResetParticle.hybridMode_, observerResetParticle.x_);
        observerArray{1}.xLocal_ = xLocal;
      end
    end
  end

  


  % for now reset observer whenever we reset the EKF
  resetObserver = resetEKF;

  % apply motion model to the estimators
  if runOptions.useEKF
    ekf.applyMotionModel(uGlobal, dt);
    % reset the EKF if necessary
    if resetEKF
      ekf.applyResetMap()
      resetEKF = false;
      ekfResetTimes(end+1) = t_current;
    end

    for i=1:num_ekfs
      % apply motion model to ekf_normal_reset and record the reset time if it occurred.
      hybridSwitch = ekfArray{i}.applyMotionModelWithHybridGuard(uGlobal, dt);
      if hybridSwitch
        ekfResetTimesArray{i}{end+1} = t_current;
      end
    end   
  end

  if runOptions.useObserver
    cgObserver.applyMotionModel(uGlobal, dt);

    % reset the EKF if necessary
    if resetObserver
      cgObserver.applyResetMap();
      resetObserver = false;
    end

    for i=1:num_observers
      hybridSwitch = observerArray{i}.applyMotionModelWithHybridGuard(uGlobal, dt);
      if hybridSwitch
        observerResetTimesArray{i}(end+1) = t_current;
        observerArray{i}.lastResetTime_ = t_current;
      end
    end
  end

  if particleFilterActive
    % also have ability to pass in options struct
    particleFilter.applyMotionModel(uGlobal, dt);
  end  

  
  t_current = t_current + dt;
  plan_time_array(end+1) = t_plan;
  
end
% profile viewer
toc;

%% Make Trajetory of true particle

simTraj = PPTrajectory(pchip(tArray, xLocalArray));
idxTraj = PPTrajectory(zoh(tArray, 1:length(tArray)));

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
plotData.hzdController = hzdController;
plotData.pdController = pdController;
plotData.trueParticleArray = trueParticleArray;
plotData.observationArray = observationArray;
plotData.times = tArray;
plotData.uArray = uArray;


if runOptions.useEKF
  plotData.kalmanFilterParticleArray = kalmanFilterParticleArray;
  plotData.kalmanFilterParticleArray = kalmanFilterBankParticleArray{1};
end

if runOptions.useParticleFilter
%   plotData.particleSetArray = particleSetArray;
  plotData.particleSetArray = particleSetArray;
end

if runOptions.useObserver
  plotData.observerParticleArray = observerBankParticleArray{1};

  % this stuff is needed for some plotControlData stuff later
  plotData.observerBankParticleArray = observerBankParticleArray;
  plotData.observerArray = observerArray;
end

% if runOptions.useObserver
%   plotData.observerParticleArray = observerParticleArray;
% end

plotParticles(plotData, fig)

% plotDataAfterResampling = plotData;
% plotDataAfterResampling.particleSetArray = particleSetArray;
% plotDataAfterResampling.plotWeights = true;
% fig = figure(figCounter);
% figCounter = figCounter + 1;

% plotParticles(plotDataAfterResampling, fig);

%% control input plot
timeIdxTraj = zoh(tArray, 1:length(tArray));


impactTime = hybridEventTimes(1);
t_min = impactTime -0.05;
t_max = impactTime + 0.2;

idx_min = ppval(timeIdxTraj, t_min);
idx_max = ppval(timeIdxTraj, t_max);

plotControlDataOptions = struct();
plotData.idxRange = idx_min:idx_max; % note this is needed by PlotUtility as well, see below

plotControlDataOptions = struct();
plotControlDataOptions.controlTypeToPlot = 'hzd';

if runOptions.useParticleFilter
  plotControlData(plotData, plotControlDataOptions)
end
%% plot trajectory data

plotData.cgLQR = cgLQR;
plotData.plan_time_array = plan_time_array;
trajPlotOptions = struct();
trajPlotOptions.plotEKF = false;
trajPlotOptions.plotLQR = false;
trajPlotOptions.controlTypeToPlot = 'hzd';


if runOptions.useObserver
  trajPlotOptions.plotObserver = true;
end

trajPlot = PlotParticleTrajectory(plotData,trajPlotOptions);
% trajPlot.plotDataArray(trajPlot.plotArray_)

trajPlot.plot(trajPlotOptions)


%% Interactive Control Plot
% figHandle_ = figure(31);
% plotData.figHandle_ = figHandle_;
% plotUtils = PlotUtility(plotData);
% plotUtils.setupPlots()







