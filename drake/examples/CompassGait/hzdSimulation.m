%% Run trajectory optimization

% setup options for the optimization/simulation
options = struct();
options.slope = 0;
options.numKnotPoints = 50;
options.plot = true;
options.u_const_across_transitions = false;


[p,utraj,xtraj,z,traj_opt, returnData] = runDircolCycleLucas(options);


v = CompassGaitVisualizer(p, xtraj.getOutputFrame);

% Visualize simulated trajectory
playback(v,xtraj,struct('slider',true));




%% Setup HZD Controller and Plant
hzdController = HZDController(p);
hzdController = hzdController.setNominalTrajectory(xtraj, utraj);

%%
s = CompassGaitPlantWithController(p.gamma);
s = s.setController(hzdController);

% Setup initial state
x0 = xtraj.eval(0);
x_perturb = 0*x0;
x_perturb(4) = x_perturb(4) - 0.2; % perturb initial velocity of swing leg

x0_hzd = x0 + x_perturb;
T = 10;
% simulate closed loop system
[ytraj_hzd, xtraj_hzd] = simulate(s, [0,T], x0_hzd);

s_vis = CompassGaitVisualizer(s);
playback(s_vis,ytraj_hzd,struct('slider',true));

%% Lyapunov function for PD controller on y

A = [0,1;0,0];
B = [0;1];

K = [hzdController.Kp,hzdController.Kd]; % the gain matrix, u = -K x

temp = A - B*K;
Q = eye(2);

% this is a Lyapunov function for our PD controller
S = lyap(temp,Q);


%% Some plotting
% everything from here downwards is just debugging

te = xtraj_hzd.te;
controlTrajs = hzdController.reconstructControlDataFromTrajectory(xtraj_hzd);
tGrid = ytraj_hzd.getBreaks();
tBreaks = tGrid;
ytraj_hzdGrid = ytraj_hzd.eval(tBreaks);

phaseSpan = hzdController.uPhaseTraj.tspan;


tControlTrajs = controlTrajs.grid.t;
xTemp = xtraj_hzd.eval(tControlTrajs);
phaseGrid = xTemp(3,:);
phaseGridClipped = max(phaseSpan(1), min(phaseSpan(2), phaseGrid));
uPlanGrid = hybridTrajEval(hzdController.uPhaseTraj, phaseGridClipped);

fig = figure(7);
clf(fig);
hold on;
plot(controlTrajs.grid.t, controlTrajs.grid.y,'b');
plot(controlTrajs.grid.t, controlTrajs.grid.ydot,'r');
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


% this is the nominal control input
plot(tControlTrajs, uPlanGrid, 'c');

% plot control input from our tape
utraj_hzd = ytraj_hzd(6);
utraj_hzdGrid = utraj_hzd.eval(tControlTrajs);
plot(tControlTrajs, utraj_hzdGrid, 'm');

lineSize = max(controlTrajs.grid.u);
for i=1:length(te)
  H = line([te(i),te(i)],[-lineSize, lineSize]);
end
% plot(controlTrajs_2.grid.t, controlTrajs_2.grid.u,'r');
title('control input');
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
% plot(controlTrajs.grid.t, controlTrajs.grid.phaseVar, 'g')

title('state trajectory');
hold off;

%% Plot Lyapunov function over time
gamma = [controlTrajs.grid.y, controlTrajs.grid.ydot];

temp = gamma.*(gamma*S);
V_grid = mean(temp,2);% this is the value of the Lyapunov function


fig = figure(6);
clf(fig);
hold on;
plot(tControlTrajs, V_grid, 'r');
lineMax = max(V_grid);
lineMin = -0.2*lineMax;

for i=1:length(te)
  H = line([te(i),te(i)],[lineMin, lineMax]);
end

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
% plot(tBreaks, hdDeriv2Grid, 'r'); % second derivative is huge actually

hold off;

% %%
% t = 0;
% x = xtraj.eval(t);
% x(2) = x(2) + 0.01;
% [u, u_ff, u_fb] = hzd.testGetControlInput(t,x);




%% Testing of mode switching controller
t = 0;
x = xtraj_hzd.eval(t)
y = ytraj_hzd.eval(t);
u = y(end)
u_global = hzdController.cgUtils.transformControlInput(x(1), u)
[x_global, d] = hzdController.cgUtils.transformStateToGlobal(x)

d_right_x = d.right.x
d_left_x = d.left.x
d_left_q = d.left.q


dynamicsData = hzdController.manipulatorDynamicsFromState(x);

dModes = hzdController.dynamicsInBothModes(x, u_global);

hzDynamics = hzdController.computeHybridZeroDynamics(x);

% disp('-------qddot------------')
% dModes.right.qddot_global
% dModes.left.qddot_global

%% Hybrid mode uncertainty testing
te = xtraj_hzd.te;
eps = 0.05;
idx = 3;

t_minus = te(idx) - eps;
t_plus = te(idx) + eps;

t = t_plus

y = ytraj_hzd.eval(t_minus);
u_minus = y(end)



y = ytraj_hzd.eval(t_plus);
u_plus = y(end)


x = xtraj_hzd.eval(t)
y = ytraj_hzd.eval(t);
u = y(end)
u_global = hzdController.cgUtils.transformControlInput(x(1),u);
% u = utraj.eval(t)

hzDynamics = hzdController.computeHybridZeroDynamics(x)
xi = [hzDynamics.y; hzDynamics.ydot];

[u1, returnData] = hzdController.testGetControlInput(t,x);

d = hzdController.computeHybridZeroDynamics(x);
Ay = d.A_y;
By = d.B_y;


d2 = hzdController.zeroDynamicsBothModes(x)
hzdRight = d2.right
hzdLeft = d2.left

yddotLeft = @(x) d2.left.A_y + d2.left.B_y*x;
yddotRight = @(x) d2.right.A_y + d2.right.B_y*x;


fig = figure(10);
clf(fig);
hold on;
fplot(yddotLeft, [-3,3], 'r');
fplot(yddotRight, [-3,3], 'b');

plot(u_global, yddotLeft(u_global), 'r*');
plot(u_global, yddotRight(u_global), 'b*');

xlabel('uGlobal');
ylabel('yddot');
title('yddot vs uGlobal');
hold off;



xi_left = [hzdLeft.y; hzdLeft.ydot];
Vdot_left = @(x) xi_left'*S*[xi_left(2); yddotLeft(x)];
V_left = xi_left'*S*xi_left

xi_right = [hzdRight.y; hzdRight.ydot];
Vdot_right = @(x) xi_right'*S*[xi_right(2); yddotRight(x)];

V_right = xi_right'*S*xi_right

fig = figure(11);
clf(fig);
hold on;
fplot(Vdot_left, [-3,3], 'r');
fplot(Vdot_right, [-3,3], 'b');

plot(u_global, Vdot_left(u_global), 'r*');
plot(u_global, Vdot_right(u_global), 'b*');
xlabel('uGlobal');
ylabel('Vdot')
title('Vdot vs. uGlobal');
hold off;

%% 

t = 9.84;
x = xtraj_hzd.eval(t)
xs = xtraj_hzd.eval(t);
xs = xs(2:end);
theta_sw = xs(1);
theta_st = xs(2);
sig = theta_sw - theta_st
p.footCollisionGuard1(t,xs,0)
[val, returnData] = hzdController.uncertaintyAboutHybridMode(t,x)


























