
%% Get trajectory of passive Compass Gait
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
gammaNew = 0*pi/180;
p = CompassGaitPlant(gammaNew);
options = struct();
options.numKnotPoints = 40;
options.plant = p;

segmentIdx = xtraj.traj{8};
xtraj_single_step = xtraj.traj{8};
xtraj_single_step_no_mode = xtraj_single_step(2:5);
[p, utraj_opt, xtraj_opt] = SingleStepOptimization(xtraj_single_step_no_mode, options);

% ytraj_single_step = ytraj.traj{8};
% 
% %% Now we transform the given trajectory to one for a compass gait on a flat slope
% s = CompassGaitPlant(0); % this is compass gait on a flat slope
% ytraj_flat = ytraj - [0;gammaIn; gammaIn; 0; 0; 0];
% 
% v_s = CompassGaitVisualizer(s);
% v_s.playback(ytraj_flat, struct('slider',true));
% 

%% Compute resulting torques
