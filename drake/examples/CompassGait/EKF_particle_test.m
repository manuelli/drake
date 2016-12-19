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


%% Extract the nominal trajectory
xtraj_single_step = xtraj_opt;

%% Setup HZD Controller
% standard, no uncertainty crap here
options = struct();
options.Kp = 50;
options.dampingRatio = 1.0;
options.useLQR = false;
options.usePlanStanceLegVelocity = true;
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
t_f = 20;

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

options = struct();
options.initializationPositionNoiseStdDev = 0.01;
options.initializationVelocityNoiseStdDev = 0.01;

options.processNoiseStdDev_q = 0.02;
options.processNoiseStdDev_v = 0.02;


options.measurementNoiseIMUVar = dt*1e-4;
options.measurementNoiseEncodersVar = dt*1e-4;
options.numParticles = 100;

truthParticleOptions = struct();
truthParticleOptions.numParticles = 0;

controllerOptions = struct();

% set the type of the controller
% options are
% - hzd
% - lqr
controllerOptions.controlStrategy = 'hzd';

% options include
% - normal
% - deadzone
% - robust
controllerOptions.type = 'normal'; % just standard controller

% what state to use in the controller
% options include
% - true
% - ekf
controllerOptions.controlState = 'ekf'; 
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



particleFilter = CompassGaitParticleFilter(plant, options);
particleFilter.nominalTraj_ = xtraj_single_step;


ekf = CompassGaitExtendedKalmanFilter(plant, options);

% determines whether or not we add noise to the measurements
observationOptions = struct();
observationOptions.addNoise = false;
observationOptions.bias = [0;0]; % a bias to add to the measurements, see how this affects performance





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
plan_time_array = [];
stanceAngleAfterReset = [];
planTimeAfterImpactArray = [];
uArray = [];
xLocalArray = [];

hybridEventTimes = [];
ekfResetTimes = [];

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

  % just command 0 if this is the first time through the loop.
  % otherwise can do uncertainty aware control input and stuff
  if ~simulatorInitialized
    uGlobal = 0;
    simulatorInitialized = true;
  else

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
    end

    xLocal = cgUtils.transformGlobalStateToLocalState(trueParticle.hybridMode_, trueParticle.x_);
    thetaTrue = xLocal(2);
    if (controllerOptions.useHackyStuff && (thetaTrue > controllerOptions.hackThetaLowerBound) && (controllerOptions.hackThetaUpperBound))
      controlStateParticle = trueParticle;
    end

    if strcmp(controllerOptions.controlStrategy, 'hzd')
      [uGlobal, controlReturnData] = hzdController.getControlInputFromGlobalState(t_current, controlStateParticle.hybridMode_, controlStateParticle.x_, controllerOptions);
    elseif strcmp(controllerOptions.controlStrategy, 'lqr')

      if (isempty(hybridEventTimes))
        t_plan = t_current;
      else
        t_plan = t_current - hybridEventTimes(end);
      end

      [uGlobal, controlReturnData] = cgLQR.getControlInputFromGlobalState(t_plan, controlStateParticle.hybridMode_, controlStateParticle.x_);
    else
      error('controller type must be one of lqr or hzd');
    end


    if (particleFilterActive)
      switch controllerOptions.type
        case 'deadzone'
          uGlobal = 0;
        case 'robust'
          uGlobal = hzdController.computeRobustControlFromParticleSet(particleFilter.particleSet_, dt);
      end
    end
  end
 
  uArray(end+1) = uGlobal;

  % now record all the estimation stuff that went along with this control
  trueParticleArray{end+1} = CompassGaitParticle.copy(trueParticle);

  if runOptions.useEKF
    kalmanFilterParticleArray{end+1} = ekf.getKalmanFilterStateAsParticle();
  end

  if runOptions.useParticleFilter
    if particleFilterActive
      particleSetArray{end+1} = CompassGaitParticle.copyParticleSet(particleFilter.particleSet_);
    else
      particleSetArray{end+1} = {};
    end
  end
  
  % move the true particle forwards using the motion model
  outputData = particleFilter.applyMotionModelSingleParticle(trueParticle,uGlobal,dt,struct('useUncertainty',false));
  
  % record any hybrid events that may have occurred
  if(isfield(outputData,'hybridEventTime'))
     hybridEventTimes(end+1) = t_current + outputData.hybridEventTime;
     hybridEvent = true;

     xLocal = cgUtils.transformGlobalStateToLocalState(trueParticle.hybridMode_, trueParticle.x_);

     stanceAngleAfterReset = xLocal(2);
     planTimeAfterImpactArray(end+1) = cgLQR.ttrajPhase.eval(stanceAngleAfterReset);
  end
  
  % move the particles in the filter using the stochastic motion model
%   particleFilter.applyMotionModel(0,dt);


  % if needed, apply the reset map to the ekf
  if (hybridEvent && strcmp(ekfOptions.resetType, 'true'))
    resetEKF = true;
    % ekf.applyResetMap();
  end
  
  % generate and observation
  [y, yParticle] = particleFilter.generateObservation(trueParticle, dt, observationOptions);

  observationArray{end+1} = yParticle; % record the observations that we had

  if runOptions.useEKF
    % for now we will tell the EKF the correct hybrid mode
    % apply the motion model
    ekf.applyMotionModel(uGlobal, dt);
    ekf.applyMeasurementUpdate(y); % note y is in global coords
    % kalmanFilterParticleArray{end+1} = ekf.getKalmanFilterStateAsParticle();
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

          if strcmp(ekfOptions.resetType, 'early')
            resetEKF = true;
          end
        end
      end
    end
    
    if (particleFilterActive)    
      % apply motion model
      particleFilter.applyMotionModel(0,dt);

      % store the particle set
      % particleSetArray{end+1} = CompassGaitParticle.copyParticleSet(particleFilter.particleSet_);

      % HACK: add truth particles if necessary
  %     particleFilter.addTruthParticles(trueParticle, truthParticleOptions);

      % apply the measurement update
      particleFilter.applyMeasurementUpdate(y,dt);
      particleFilter.applyImportanceResampling();

    % store the particle set after importance resampling
      % particleSetArrayAfterImportanceResampling{end+1} = CompassGaitParticle.copyParticleSet(particleFilter.particleSet_);
    end

    
    
    % deactivate PF when all in the next mode
    if (particleFilterActive && (trueParticle.hybridMode_ ~= pfActivationMode))
      if ((particleFilter.particleSet_{1}.hybridMode_ ~= pfActivationMode) && CompassGaitParticle.allParticlesInSameMode(particleFilter.particleSet_))
        disp('deactivating PF');
        particleFilterActive = false;

        if strcmp(ekfOptions.resetType, 'late')
          resetEKF = true;
        end
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

  if resetEKF
    disp('resetting EKF');
    ekf.applyResetMap();
    ekfResetTimes(end+1) = t_current;
    resetEKF = false;
  end

  t_current = t_current + dt;
  tArray(end+1) = t_current;
  plan_time_array(end+1) = t_plan;
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
plotData.hzdController = hzdController;
plotData.trueParticleArray = trueParticleArray;
plotData.observationArray = observationArray;
plotData.times = tArray;
plotData.uArray = uArray;

if runOptions.useEKF
  plotData.kalmanFilterParticleArray = kalmanFilterParticleArray;
end

if runOptions.useParticleFilter
%   plotData.particleSetArray = particleSetArray;
  plotData.particleSetArray = particleSetArray;
end

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
trajPlot = PlotParticleTrajectory(plotData,trajPlotOptions);
% trajPlot.plotDataArray(trajPlot.plotArray_)

trajPlot.plot(trajPlotOptions)
%% Interactive Control Plot
% figHandle_ = figure(31);
% plotData.figHandle_ = figHandle_;
% plotUtils = PlotUtility(plotData);
% plotUtils.setupPlots()







