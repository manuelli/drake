
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

% specify which controller should run when there is uncertainty
options.applyDeadZoneController = false;
options.applyWrongModeController = false;


% specify when you have the mode uncertainty, before contact, after contact
% or both
options.applyUncertaintyControllerOnMakingContact = true;
options.applyUncertaintyControllerOnBreakingContact = false;

% specify how much mode uncertainty you have
options.hybridModeUncertainty.makingContactToeHeightThreshold = 0.03;
options.hybridModeUncertainty.breakingContactTimeThreshold = 0.1;

hzdController = HZDController(p, options);
hzdController = hzdController.setNominalTrajectory(xtraj_opt, utraj_opt);


%% Run the simulation
s = TimeSteppingCompassGaitPlant(p.gamma);

x0 = [xtraj_opt.eval(0.3)];

swingVelDelta = 0.0;
stanceVelDelta = 0.0;
x0_delta = [0;0;swingVelDelta;stanceVelDelta];

x0_sim = [1;x0 + x0_delta]; % start in mode 1
tspan = [0,3];
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

%% Some more analysis stuff for the other mode
controlDataOtherCellArray = {};
for i = 1:length(d.tControlGrid)
   t = d.tControlGrid(i);
   x = xtraj.eval(t);
   xOther = hzdController.cgUtils.transformStateToOtherMode(x);
   
   cData = hzdController.getStandardControlInput(xOther);
   controlDataOtherCellArray{end+1} = cData;
end

controlDataOtherTrajs = hzdController.makeTrajectoriesFromCellArray(d.tControlGrid, controlDataOtherCellArray);

%% Some plotting
controlDataTrajs = inputStruct.controlTrajs.controlData;
fig = figure(15);
clf(fig);
fnplt(controlDataTrajs.alphaVal);

figCounter = 15;
fig = figure(figCounter);
clf(fig);
hold on;
tspanPlot = [0.4, 0.6];

h = fnplt(controlDataTrajs.u_mode_1.trim(tspanPlot));
set(h, 'Color', 'b');


h = fnplt(controlDataTrajs.u_mode_2.trim(tspanPlot));
set(h, 'Color', 'r');

h = fnplt(controlDataTrajs.u_simple_blend.trim(tspanPlot));
set(h,'Color','g');
hold off;
figCounter = figCounter + 1;


fig = figure(figCounter);
clf(fig);
hold on;

h = fnplt(controlDataTrajs.y.trim(tspanPlot));
set(h, 'Color', 'b');


h = fnplt(controlDataTrajs.ydot.trim(tspanPlot));
set(h, 'Color', 'g');
% 
% h = fnplt(controlDataTrajs.u_simple_blend.trim(tspanPlot));
% set(h,'Color','g');
hold off;

figCounter = figCounter + 1;



fig = figure(figCounter);
clf(fig);
hold on;

h = fnplt(controlDataOtherTrajs.y.trim(tspanPlot));
set(h, 'Color', 'b');


h = fnplt(controlDataOtherTrajs.ydot.trim(tspanPlot));
set(h, 'Color', 'r');


h = fnplt(controlDataOtherTrajs.yddot.trim(tspanPlot));
set(h, 'Color', 'g');

% h = fnplt(controlDataOtherTrajs.u.trim(tspanPlot));
% set(h, 'Color', 'c');
% 
% h = fnplt(controlDataTrajs.u_simple_blend.trim(tspanPlot));
% set(h,'Color','g');
hold off;
title('y in blue, ydot in red, yddot in green for wrong mode');
figCounter = figCounter + 1;


fig = figure(figCounter);
clf(fig);
hold on;
% 
h = fnplt(controlDataOtherTrajs.A_y.trim(tspanPlot));
set(h, 'Color', 'b');


h = fnplt(controlDataOtherTrajs.B_y.trim(tspanPlot));
set(h, 'Color', 'r');
hold off;

title('A_y in blue, B_y in red for wrong mode');

figCounter = figCounter + 1;

fig = figure(figCounter);
clf(fig);
hold on;

h = fnplt(controlDataTrajs.A_y.trim(tspanPlot));
set(h, 'Color', 'b');


h = fnplt(controlDataTrajs.B_y.trim(tspanPlot));
set(h, 'Color', 'r');
hold off;

title('A_y in blue, B_y in red for correct mode');

figCounter = figCounter + 1;

%%

t = 0.5;
x = xtraj.eval(t)

xOther = hzdController.cgUtils.transformStateToOtherMode(x);

cData = hzdController.getStandardControlInput(xOther)






























