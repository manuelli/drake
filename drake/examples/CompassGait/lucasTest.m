r = CompassGaitPlant();
s = CompassGaitPlantWithController();
% r.useFixedOutputCoords = true;
x0 = r.getInitialState;
T = 5;

% %% testing
% s.update(0,x0,0)
% s.dynamics(0,x0,0)
% s.output(0,x0,0);

%% simulation without feedback
[ytraj, xtraj] = simulate(r, [0, T], x0);
% v = CompassGaitVisualizer(s);
% v.playback(ytraj, struct('slider',true));

%% Setup TVLQR type controller
c = CompassGaitController(r);
c = c.setNominalTrajectory(ytraj);
c = c.setupLQRControllers();

%% Simulate with Feedback
s = s.setController(c);
x0_perturb = x0;
x0_perturb(4) = x0(4) + 0.01;
[ytraj_sys, xtraj_sys] = simulate(s, [0, T], x0_perturb);
v = CompassGaitVisualizer(s);
playback(v,ytraj_sys,struct('slider',true));

%% Testing
s.update(0,x0,0)
s.dynamics(0,x0,0)
s.output(0,x0,0)

% %%
% t = 0.0;
% x0 = xtraj.eval(t);
% x0(4) = x0(4) + 0.1;
% x0
% c = c.step(t,x0);
% c.getCurrentControlInput()
% 
% c.controlInputFromPlantState(t, xtraj.eval(t))

%% Plotting
tGrid = linspace(0,T,100);
yGrid = ytraj_sys.eval(tGrid);
yGrid_des = ytraj_sys.eval(tGrid);

uGrid = yGrid(end,:);
modeGrid_des = yGrid_des(1,:);
modeGrid = yGrid(1,:);

figure(1)
plot(tGrid,uGrid,'g')
plot(tGrid,
