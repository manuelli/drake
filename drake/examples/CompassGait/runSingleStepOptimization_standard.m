
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
% v.playback(ytraj, struct('slider',true));


%% Optimization with the plant we want
gammaOpt = 0*pi/180;
p = CompassGaitPlant(gammaOpt);
options = struct();
options.numKnotPoints = 40;
options.plant = p;
options.u_const_across_transitions = false;
options.stanceLegSweepAngleLowerBound = 0.25;

segmentIdx = xtraj.traj{8};
xtraj_single_step = xtraj.traj{8};
xtraj_single_step_no_mode = xtraj_single_step(2:5);
[p, utraj_opt, xtraj_opt] = SingleStepOptimization(xtraj_single_step_no_mode, options);


%% Setup HZD Controller
options = struct();
options.Kp = 20;
options.dampingRatio = 1.0;

% set this to true if you want to run the dead-zone controller
options.applyDeadZoneController = false;

options.hybridModeUncertainty.makingContactToeHeightThreshold = 0.05;

hzdController = HZDController(p, options);
hzdController = hzdController.setNominalTrajectory(xtraj_opt, utraj_opt);

%% Plant for simulation
s = CompassGaitPlantWithController(p.gamma);
s = s.setController(hzdController);
vis_temp = CompassGaitVisualizer(s, xtraj_opt.getOutputFrame);

% playback the  the optimized trajectory
playback(vis_temp,xtraj_opt,struct('slider',true));



%% Plot some of the hzd stuff
fig = figure(3);
clf(fig);
hold on;
fnplt(hzdController.hdTraj);
title('h_d(theta)');
hold off;

fig = figure(4);
clf(fig);
hold on;
fnplt(hzdController.hdTraj_deriv);
title('h_d deriv(theta)');

hold off;

fig = figure(5);
clf(fig);
hold on;
fnplt(hzdController.hdTraj_dderiv);
title('h_d dderiv(theta)');

hold off;

fig = figure(6);
clf(fig);
hold on;
fnplt(hzdController.uPhaseTraj);
title('u(theta)');
hold off;





%% Simulate from a variety of initial conditions
% start halfway through swing
x0 = xtraj_opt.eval(0.3);
T = 10;

simDataCells = {};
stanceVelDelta = [-0.1:0.05:0.1];
swingVelDelta = [-0.2; 0.1; 0.2];

stanceVelDelta = [0.0];
swingVelDelta = [0.0; -0.2];

if (false)
  % hack to only run a single simulation instead of many
  stanceVelDelta = 0.0;
  swingVelDelta = 0.0;
end


numRows = length(swingVelDelta);
numCols = length(stanceVelDelta);

counter = 1;

for i=1:numRows
  for j=1:numCols
    x0_delta = [0;0;swingVelDelta(i);stanceVelDelta(j)]
    simData = struct();
    x0_sim = [1; x0 + x0_delta];
    simData.x0_delta = x0_delta;
    [ytraj_sim, xtraj_sim] = simulate(s, [0,T], x0_sim);

    simData.ytraj = ytraj_sim;
    simData.xtraj = xtraj_sim;

    simData.fell = CompassGaitUtils.doesTrajectoryFall(xtraj_sim);

    if (simData.fell)
      disp('Robot Fell with initial perturbation')
      x0_delta
    end
    
    simDataCells{counter} = simData;
    counter = counter + 1;
  end
end




%% Plot some results of the simulation

inputStruct = simDataCells{2};
inputStruct.plant = s;
x0_delta = inputStruct.x0_delta;

options.allPlots = true;

controlTrajs = analyzeTrajectory(inputStruct,options);
xtraj = inputStruct.xtraj;

%% Some debugging stuff
disp('--------------')
t = 0.35
x = xtraj.eval(t)
toeHeight = p.computeToeHeight(x)
guard1 = p.footCollisionGuard1(t,x(2:end),0)
guard2 = p.footCollisionGuard2(t,x(2:end),0)





%% Some optional plotting

tspan = [2,8];
tGrid = linspace(tspan(1),tspan(2), 1000);
valGrid = controlTrajs.controlData.V.eval(tGrid);

figCounter = 10;
fig = figure(figCounter);
clf(fig);
plot(tGrid, valGrid);
































