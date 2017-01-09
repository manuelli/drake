%% Get trajectory of passive Compass Gait
% This is  in order to a pass a reasonable initial trajectory to the trajectory optimization
% First we simulate a passive compass gait on a slope
close all;
gammaIn = 3*pi/180;
r = CompassGaitPlant(gammaIn);

x0 = r.getInitialState;
T = 10;


%% simulation without feedback
[ytraj, xtraj] = simulate(r, [0, T], x0);
v = CompassGaitVisualizer(r);


%% Optimize a trajectory for flat ground
% gammaOpt = 0*pi/180;
gammaOpt = gammaIn; % do it on a zero slope
p = CompassGaitPlant(gammaOpt);
options = struct();
options.numKnotPoints = 40;
options.plant = p;
options.u_const_across_transitions = false;
options.stanceLegSweepAngleLowerBound = 0.25;
options.useDeltaUCost = true;
options.deltaUCostWeight = 3.0;

segmentIdx = xtraj.traj{8};
xtraj_single_step = xtraj.traj{8};
xtraj_single_step_no_mode = xtraj_single_step(2:5);
[p, utraj_opt, xtraj_opt] = SingleStepOptimization(xtraj_single_step_no_mode, options);


gammaVals = [3];
plotNominalTrajectory(xtraj_opt, gammaVals);

%% Extract the nominal trajectory
xtraj_single_step = xtraj_opt;

%% Setup HZD Controller
% standard, no uncertainty crap here
options = struct();
options.Kp = 50;
options.dampingRatio = 1.0;
options.useLQR = false;
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
cgUtils = CompassGaitUtils();
t_start = 0.3;
t = t_start;
dt = 0.0025;
x_local_orig = xtraj_opt.eval(t);

delta_x = [0;0;-0.1;0];
x_local = x_local_orig + delta_x;


hybridMode = 1;
t_f = 2;

numTimesteps = ceil((t_f-t)/dt);
t_f = t + numTimesteps*dt;
% x_final = xtraj.eval(t_f);
xGlobal = cgUtils.transformLocalStateToGlobalState(hybridMode, x_local);

t_current = t;

% x_final_global = cgUtils.transformLocalStateToGlobalState(x_final(1), x_final(2:end));

%% Setup the options for EKF and PF

runOptions = struct();
runOptions.useParticleFilter = false;
runOptions.useEKF = true;
runOptions.useObserver = true;


% these options apply to the EKF and also to the Particle Filter
options = struct();
options.initializationPositionNoiseStdDev = 0.01;
options.initializationVelocityNoiseStdDev = 0.01;

options.processNoiseStdDev_q = 0.002;
options.processNoiseStdDev_v = 0.002;


options.measurementNoiseIMUVar = dt*0.00;
options.measurementNoiseEncodersVar = dt*1e-4;
options.numParticles = 100;

truthParticleOptions = struct();
truthParticleOptions.numParticles = 0;

controllerOptions = struct();

% set the type of the controller
% options are
% - hzd
% - lqr
% - pd
controllerOptions.controlStrategy = 'pd';

% options include
% - normal
% - deadzone
% - robust
controllerOptions.type = 'normal'; % just standard controller

% what state to use in the controller
% options include
% - true
% - ekf
% - observer
controllerOptions.controlState = 'true'; 
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
cgObserverOptions.epsilon = 0.2;
cgObserver = CompassGaitObserver(plant, cgObserverOptions);
cgObserver.initialize(hybridMode, xGlobal);

%% Setup Particle Filter

particleFilter = CompassGaitParticleFilter(plant, options);
particleFilter.nominalTraj_ = xtraj_single_step;

%% Setup EKF
% standard one
ekf = CompassGaitExtendedKalmanFilter(plant, options);

% this ekf has a built in reset map, i.e. it uses the reset map of the plant
ekf_normal_reset = CompassGaitExtendedKalmanFilter(plant, options);

% determines whether or not we add noise to the measurements
observationOptions = struct();
observationOptions.addNoise = false;
observationOptions.bias = [0;0]; % a bias to add to the measurements, see how this affects performance





%% Test the particle filter functionality

if runOptions.useParticleFilter
  particleFilter.initializeFilter(hybridMode, xGlobal);
end

ekf.initializeFilter(hybridMode, xGlobal);
ekf_normal_reset.initializeFilter(hybridMode, xGlobal);


inputData = struct('xGlobal',xGlobal);
inputData.hybridMode = hybridMode;
particle = CompassGaitParticle(inputData);
trueParticle = CompassGaitParticle(inputData);

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

ekfResetTimes_normal_reset = [];

uGlobal = 0;
simulatorInitialized = false;

particleFilterActive = false;
particleFilter.particleSet_ = {};

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

    ekf_normal_reset.applyMeasurementUpdate(y);
    kalmanFilterParticleArray_normal_reset{end+1} = ekf_normal_reset.getKalmanFilterStateAsParticle();
  end

  if runOptions.useObserver
    cgObserver.applyMeasurementUpdate(y, dt);
    observerParticleArray{end+1} = cgObserver.getObserverStateAsParticle();
  end

  if runOptions.useParticleFilter
    if particleFilterActive
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
    controlStateParticle = cgObserver.getObserverStateAsParticle();
  end
    

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
  if (particleFilterActive)
    switch controllerOptions.type
      case 'deadzone'
        uGlobal = 0;
      case 'robust'
        uGlobal = hzdController.computeRobustControlFromParticleSet(particleFilter.particleSet_, dt);
    end
  end
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

          if strcmp(ekfOptions.resetType, 'early')
            resetEKF = true;
          end
        end
      end
    end
  end

  % decide whether we should de-activate particle filter
  if (runOptions.useParticleFilter && particleFilterActive)
    % all particles must be in the same mode, and that mode must be different than the 
    % mode in which the PF was initialized
    if CompassGaitParticle.allParticlesInSameMode(particleFilter.particleSet_)
      currentMode = particleFilter.particleSet_{1}.hybridMode_;
      if currentMode ~= pfActivationMode
        particleFilterActive = false;
        disp('de-activating particle filter')

        % trigger the 'late' EKF reset
        if strcmp(ekfOptions.resetType, 'late')
          resetEKF = true;
        end
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

    % apply motion model to ekf_normal_reset and record the reset time if it occurred.
    hybridSwitch = ekf_normal_reset.applyMotionModelWithHybridGuard(uGlobal, dt);
    if hybridSwitch
      ekfResetTimes_normal_reset(end+1) = t_current;
    end
  end

  if runOptions.useObserver
    cgObserver.applyMotionModel(uGlobal, dt);

    % reset the EKF if necessary
    if resetObserver
      cgObserver.applyResetMap();
      resetObserver = false;
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
  plotData.kalmanFilterParticleArray = kalmanFilterParticleArray_normal_reset;
end

if runOptions.useParticleFilter
%   plotData.particleSetArray = particleSetArray;
  plotData.particleSetArray = particleSetArray;
end

if runOptions.useObserver
  plotData.observerParticleArray = observerParticleArray;
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
if runOptions.useParticleFilter
  plotData.idxRange = 97:114;
  plotControlData(plotData)
end
%% plot trajectory data

plotData.cgLQR = cgLQR;
plotData.plan_time_array = plan_time_array;
trajPlotOptions = struct();
trajPlotOptions.plotEKF = true;
trajPlotOptions.plotLQR = false;
trajPlotOptions.controlTypeToPlot = 'pd';


if runOptions.useObserver
  trajPlotOptions.plotObserver = false;
end

trajPlot = PlotParticleTrajectory(plotData,trajPlotOptions);
% trajPlot.plotDataArray(trajPlot.plotArray_)

trajPlot.plot(trajPlotOptions)
%% Interactive Control Plot
% figHandle_ = figure(31);
% plotData.figHandle_ = figHandle_;
% plotUtils = PlotUtility(plotData);
% plotUtils.setupPlots()







