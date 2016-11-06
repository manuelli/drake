
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


%% Run the simulation
s = TimeSteppingCompassGaitPlant(p.gamma);

x0 = [xtraj_opt.eval(0.3)];

swingVelDelta = -0.2;
stanceVelDelta = 0.0;
x0_delta = [0;0;swingVelDelta;stanceVelDelta];

x0_sim = [1;x0 + x0_delta]; % start in mode 1
tspan = [0,2];
dt = 0.01; % controller is running at 100Hz here
startTime = tic;
d = TimeSteppingSimulationWithController(s, hzdController, tspan, dt, x0_sim); 
elapsedTime = toc(startTime)
v = CompassGaitVisualizer(s, d.xtraj.getOutputFrame);
v.playback(d.xtraj, struct('slider',true));

xtraj = d.xtraj;


%%
trajsTemp = hzdController.makeTrajectoriesFromCellArray(d.tControlGrid, d.controlDataCellArray);

controlTrajs = struct();
controlTrajs.controlData = trajsTemp;

inputStruct = struct();
inputStruct.plant = s;
inputStruct.xtraj = d.xtraj;
inputStruct.controlTrajs = controlTrajs;

plotOptions = struct();
plotOptions.allPlots = true;
analyzeTrajectory(inputStruct, plotOptions);

%% Some plotting
controlDataTrajs = inputStruct.controlTrajs.controlData;
fig = figure(15);
clf(fig);
fnplt(controlDataTrajs.alphaVal);


fig = figure(16);
clf(fig);
hold on;
newtspan = [0.4, 0.6];

h = fnplt(controlDataTrajs.u_mode_1.trim(tspanPlot));
set(h, 'Color', 'b');


h = fnplt(controlDataTrajs.u_mode_2.trim(tspanPlot));
set(h, 'Color', 'r');

h = fnplt(controlDataTrajs.u_simple_blend.trim(tspanPlot));
set(h,'Color','g');
hold off;

































