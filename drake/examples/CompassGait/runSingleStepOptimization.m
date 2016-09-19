
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
gammaOpt = 0*pi/180;
p = CompassGaitPlant(gammaOpt);
options = struct();
options.numKnotPoints = 40;
options.plant = p;
options.u_const_across_transitions = true;

segmentIdx = xtraj.traj{8};
xtraj_single_step = xtraj.traj{8};
xtraj_single_step_no_mode = xtraj_single_step(2:5);
[p, utraj_opt, xtraj_opt] = SingleStepOptimization(xtraj_single_step_no_mode, options);


%% Setup HZD Controller
hzdController = HZDController(p);
hzdController = hzdController.setNominalTrajectory(xtraj_opt, utraj_opt);

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



%% Plant for simulation
s = CompassGaitPlantWithController(p.gamma);
s = s.setController(hzdController);

%% Simulate
x0 = xtraj_opt.eval(0.3);

x0_temp = x0 + [0;0;0.1;0];
x0_hzd = [1;x0_temp]; % need to add the discrete initial hybrid mode
T = 10;

[ytraj_hzd, xtraj_hzd] = simulate(s, [0,T], x0_hzd);

s_vis = CompassGaitVisualizer(s);
playback(s_vis,ytraj_hzd,struct('slider',true));
controlTrajs = hzdController.reconstructControlDataFromTrajectory(xtraj_hzd);



%% Plot some results of the simulation

fig = figure(7);
hold on;
h = fnplt(controlTrajs.traj.y);
set(h,'Color','b')
h = fnplt(controlTrajs.traj.ydot);
set(h,'Color','r')


fig = figure(8);
clf(fig);
hold on;
h = fnplt(controlTrajs.traj.u);
set(h, 'Color', 'b');

h = fnplt(controlTrajs.traj.u_fb);
set(h, 'Color', 'r');

h = fnplt(controlTrajs.traj.uStar);
set(h, 'Color', 'g');
hold off;


fig = figure(9);
clf(fig);
hold on;
h = fnplt(xtraj_hzd,[2]);
set(h, 'Color','b')

h = fnplt(xtraj_hzd,[3]);
set(h, 'Color','r')
title('position trajectories')
hold off;



fig = figure(10);
clf(fig);
hold on;
h = fnplt(xtraj_hzd,[3]);
set(h, 'Color','b')

h = fnplt(xtraj_hzd,[4]);
set(h, 'Color','r')
title('velocity trajectories')
hold off;

