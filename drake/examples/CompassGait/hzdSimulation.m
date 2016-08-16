%% Run trajectory optimization

slope = 3;
[p,utraj,xtraj,z,traj_opt, returnData]=runDircolCycleLucas(slope);
xtrajPhase = returnData.xtrajPhase;
utrajPhase = returnData.utrajPhase;
xdotTrajPhase = returnData.xdotTrajPhase;
xdotTraj = returnData.xdotTraj;

%% Run simulation for given number of seconds with controller
% see if controller has state or not

p = CompassGaitPlant();

% No controller, just open loop
% x0 = xtraj.eval(0);

% This is current value
x0 = [1.0000;
         0;
    0.0192;
    2.0232;
   -0.4196];
T = 5; % simulate for 5 seconds
[ytraj_sim, xtraj_sim] = simulate(p,[0,T], x0);


% Visualize simulated trajectory
v = CompassGaitVisualizer(p);
playback(v,ytraj_sim,struct('slider',true));


%% Run simulation with a controller in the loop
r = CompassGaitPlantWithController(p.gamma);
c = SimpleController(p); % I think this controller just gives a zero all the time
r = r.setController(c);
[ytraj_2, xtraj_2] = simulate(r,[0,T],x0);

r_vis = CompassGaitVisualizer(r);
playback(r_vis,ytraj_2,struct('slider',true));


%% Setup HZD Controller and Plant
hzdController = HZDController(p);
hzdController = hzdController.setNominalTrajectory(xtraj);
hzdController.Kd = 6;

%%
s = CompassGaitPlantWithController(p.gamma);
s = s.setController(hzdController);

% Setup initial state
x0_hzd = x0;
T = 20;
% simulate closed loop system
[ytraj_hzd, xtraj_hzd] = simulate(s, [0,T], x0_hzd);

s_vis = CompassGaitVisualizer(s);
playback(s_vis,ytraj_hzd,struct('slider',true));

%% Plot control input across time
tGrid = ytraj_hzd.getBreaks();
ytraj_hzdGrid = ytraj_hzd.eval(tGrid);


tGrid2 = utraj.getBreaks();
uGrid = utraj.eval(tGrid2);

fig = figure(1);
clf(fig);
hold on;
plot(tGrid, ytraj_hzdGrid(end,:), 'r');
plot(tGrid2, uGrid, 'b');
title('control input')
xlabel('time');
ylabel('u');
hold off

%%
te = xtraj_hzd.te;
controlTrajs = hzdController.reconstructControlDataFromTrajectory(xtraj_hzd);
controlTrajs_2 = hzdController.reconstructControlDataFromTrajectory(xtraj_2);


fig = figure(2);
clf(fig);
hold on;
plot(controlTrajs.grid.t, controlTrajs.grid.y,'b');
% plot(controlTrajs_2.grid.t, controlTrajs_2.grid.y,'b--');
plot(controlTrajs.grid.t, controlTrajs.grid.ydot,'r');
% plot(controlTrajs_2.grid.t, controlTrajs_2.grid.ydot,'r--');
lineSize = max(controlTrajs.grid.ydot);
for i=1:length(te)
  H = line([te(i),te(i)],[-lineSize, lineSize]);
end
title('y and ydot');
hold off;


fig = figure(3);
clf(fig);
hold on;
plot(controlTrajs.grid.t, controlTrajs.grid.u,'b');
plot(controlTrajs.grid.t, controlTrajs.grid.u_fb,'r');
plot(controlTrajs.grid.t, controlTrajs.grid.uStar,'g');

lineSize = max(controlTrajs.grid.u);
for i=1:length(te)
  H = line([te(i),te(i)],[-lineSize, lineSize]);
end
% plot(controlTrajs_2.grid.t, controlTrajs_2.grid.u,'r');
hold off;

fig = figure(4);
clf(fig);
hold on;
plot(tGrid, ytraj_hzdGrid(2,:),'b');
plot(tGrid, ytraj_hzdGrid(4,:), 'b--');
plot(tGrid, ytraj_hzdGrid(3,:),'r');
plot(tGrid, ytraj_hzdGrid(5,:), 'r--');
for i=1:length(te)
  line([te(i),te(i)],[-1,1]);
end

phaseSpan = hzdController.hdTraj.tspan;
% plot(tGrid, phaseSpan(1), 'r*');
% plot(tGrid, phaseSpan(2), 'r*');
plot(controlTrajs.grid.t, controlTrajs.grid.phaseVar, 'g')

hold off;
%% Testing of HZD Controller


tBreaks = hzdController.hdTraj.getBreaks();
tspan = hzdController.hdTraj.tspan;
tBreaks = linspace(tspan(1),tspan(2),200);
hdGrid = hzdController.hdTraj.eval(tBreaks);
hdDerivGrid = hzdController.hdTraj_deriv.eval(tBreaks);
hdDeriv2Grid = hzdController.hdTraj_dderiv.eval(tBreaks);


fig = figure(5);
clf(fig);
hold on;
plot(tBreaks, hdGrid);
plot(tBreaks, hdDerivGrid, 'r');
% plot(tBreaks, hdDeriv2Grid, 'r');

hold off;

% %%
% t = 0;
% x = xtraj.eval(t);
% x(2) = x(2) + 0.01;
% [u, u_ff, u_fb] = hzd.testGetControlInput(t,x);









%%
idx = 54;
disp('----------------')
disp('t')
t = controlTrajs.grid.t;
time = t(idx)

disp('---------')
disp('u')
controlTrajs.grid.ydot(idx)


x = xtraj_hzd.eval(time)
phase = x(2)
hzdController.hdTraj_deriv.eval(phase)
hzdController.testGetControlInput(t,x)


































