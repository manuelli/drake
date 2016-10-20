
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
v.playback(ytraj, struct('slider',true));

% extract trajectory for testing
xtraj_single_step = xtraj.traj{5};
xtraj_single_step = xtraj_single_step(2:end);

ytraj_single_step = ytraj.traj{5};
utraj_single_step = ytraj_single_step(end);

%% Simulation without feedback but with LQR controller in loop
% just do this for testing piping

lqrController = LQRController(r);
lqrController = lqrController.setNominalTrajectory(xtraj_single_step,utraj_single_step);



s = CompassGaitPlantWithController(r.gamma);
s = s.setController(lqrController);


%% Optimization with the plant we want
gammaOpt = 0*pi/180;
p = CompassGaitPlant(gammaOpt);
options = struct();
options.numKnotPoints = 40;
options.plant = p;
options.u_const_across_transitions = true;
options.stanceLegSweepAngleLowerBound = 0.25;

segmentIdx = xtraj.traj{8};
xtraj_single_step = xtraj.traj{8};
xtraj_single_step_no_mode = xtraj_single_step(2:5);
[p, utraj_opt, xtraj_opt] = SingleStepOptimization(xtraj_single_step_no_mode, options);


%% Test the LQR controller
lqrController = LQRController(p);
lqrController = lqrController.setNominalTrajectory(xtraj_opt, utraj_opt);

%% Some testing of LQR controller
t = 0.5
x = xtraj_opt.eval(t);
xFull = [1;x];
u = lqrController.computeControlInput(t,xFull, t)
u_des = lqrController.utraj.eval(t);

%%


%% Plant for simulation
s = CompassGaitPlantWithController(p.gamma);
s = s.setController(lqrController);

%% Testing
t_plan = 0.5
x = xtraj_opt.eval(t_plan);
t = 1.0
xFull = [2;x];
u = lqrController.computeControlInput(t,xFull, t_plan)
u_des = lqrController.utraj.eval(t);
s.controller.step(t,xFull,0);
s.controller.getCurrentControlInput()

%% Run a simulation
% x0 = xtraj_opt.eval(0.3);
% T = 10;
% x0_sim = [1;x0];
% 
% [ytraj_sim, xtraj_sim] = simulate(s, [0,T], x0_sim);


