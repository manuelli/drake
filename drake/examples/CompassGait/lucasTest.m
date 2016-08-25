r = CompassGaitPlant();
s = CompassGaitPlantWithController(r.gamma);
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
c.controlBasedOnSensedMode = false;

%% Simulate with Feedback
s = s.setController(c);
x0_perturb = x0;
x0_perturb(4) = x0(4) + 0.1;
[ytraj_sys, xtraj_sys] = simulate(s, [0, T], x0_perturb);
v = CompassGaitVisualizer(s);
playback(v,ytraj_sys,struct('slider',true));


%% Plotting
tGrid = linspace(0,T,500);
yGrid = ytraj_sys.eval(tGrid);
yGrid_des = ytraj.eval(tGrid);

uGrid = yGrid(end,:);
modeGrid_des = yGrid_des(1,:);
modeGrid = yGrid(1,:);

fig = figure(1);
clf(fig);
hold on;
plot(tGrid,uGrid,'g');
plot(tGrid,modeGrid_des,'b');
plot(tGrid,modeGrid,'r');
legend('u','mode des','mode actual');
hold off

fig = figure(2);
clf(fig);

hold on;
plot(tGrid, yGrid_des(2,:), 'b');
plot(tGrid, yGrid(2,:), 'r');
legend('theta1 des','theta1');
hold off;


fig = figure(3);
clf(fig);
hold on
theta1Err = yGrid(2,:) - yGrid_des(2,:);
plot(tGrid,theta1Err,'b')
title('theta1 error')
hold off

%% Transform to some global coords
traj = ytraj_sys;
outputFrame = traj.getOutputFrame();
globalCompassGaitFrame = CoordinateFrame('GlobalCompassGaitFrame',6,'x',{'mode','thetaSwing','thetaStance','thetaSwing_dot','thetaStance_dot','uGlobal'});

hybridToGlobalTransform = CompassGaitHybridToGlobalTransform(outputFrame, globalCompassGaitFrame);

outputFrame.addTransform(hybridToGlobalTransform);

trajGlobalCoord = traj.inFrame(globalCompassGaitFrame);
trajGlobalCoordGrid = trajGlobalCoord.eval(tGrid);

fig = figure(4);
clf(fig);

hold on;
plot(tGrid, trajGlobalCoordGrid(2,:), 'b');
plot(tGrid, trajGlobalCoordGrid(4,:), 'r');
legend('thetaLeft', 'thetaLeft dot');
hold off;

fig = figure(5);
clf(fig);

hold on;
plot(tGrid, trajGlobalCoordGrid(end,:), 'b');
legend('uGlobal');
hold off;

%% Testing
t = 0.42;
[t_plan, idx] = c.getClosestPlanTimeInSameMode(t,ytraj_sys.eval(t))