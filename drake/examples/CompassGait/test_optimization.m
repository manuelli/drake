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

segmentIdx = xtraj.traj{8};
xtraj_single_step = xtraj.traj{8};
xtraj_single_step_no_mode = xtraj_single_step(2:5);
[p, utraj_opt, xtraj_opt] = SingleStepOptimizationUncertainTerrainHeight(xtraj_single_step_no_mode, options);

