
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
options.applyWrongModeController = true;


% specify when you have the mode uncertainty, before contact, after contact
% or both
options.applyUncertaintyControllerOnMakingContact = false;
options.applyUncertaintyControllerOnBreakingContact = true;

% specify how much mode uncertainty you have
options.hybridModeUncertainty.makingContactToeHeightThreshold = 0.05;
options.hybridModeUncertainty.breakingContactTimeThreshold = 0.1;


hzdController = HZDController(p, options);
hzdController = hzdController.setNominalTrajectory(xtraj_opt, utraj_opt);

%% Plant for simulation
s = CompassGaitPlantWithController(p.gamma);
s = s.setController(hzdController);
vis_temp = CompassGaitVisualizer(s, xtraj_opt.getOutputFrame);

% playback the  the optimized trajectory
playback(vis_temp,xtraj_opt,struct('slider',true));



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





%% Simulate from a variety of initial conditions
% start halfway through swing
x0 = xtraj_opt.eval(0.3);
T = 10;

simDataCells = {};
stanceVelDelta = [-0.1:0.05:0.1];
swingVelDelta = [-0.2; 0.1; 0.2];

stanceVelDelta = [0.0];
swingVelDelta = [0.0, -0.2];

if (false)
  % hack to only run a single simulation instead of many
  stanceVelDelta = 0.0;
  swingVelDelta = 0.0;
end


numRows = length(swingVelDelta);
numCols = length(stanceVelDelta);

counter = 1;

for i=1:numRows
  for j=1:numCols
    x0_delta = [0;0;swingVelDelta(i);stanceVelDelta(j)]
    simData = struct();
    x0_sim = [1; x0 + x0_delta];
    simData.x0_delta = x0_delta;
    [ytraj_sim, xtraj_sim] = simulate(s, [0,T], x0_sim);

    simData.ytraj = ytraj_sim;
    simData.xtraj = xtraj_sim;

    simData.fell = CompassGaitUtils.doesTrajectoryFall(xtraj_sim);

    if (simData.fell)
      disp('Robot Fell with initial perturbation')
      x0_delta
    end
    
    simDataCells{counter} = simData;
    counter = counter + 1;
  end
end




%% Plot some results of the simulation

inputStruct = simDataCells{2};
inputStruct.plant = s;
x0_delta = inputStruct.x0_delta;

options.allPlots = true;

controlTrajs = analyzeTrajectory(inputStruct,options);
xtraj = inputStruct.xtraj;

%% Some debugging stuff


t_contact = xtraj.te(2)
tGrid = linspace(t_contact, t_contact + 0.25, 20);


V_tilde_grid = controlTrajs.controlDataOther.V.eval(tGrid);
V_tilde_dot_grid = controlTrajs.controlDataOther.V_dot.eval(tGrid);


figCounter = 10;
fig = figure(figCounter);
clf(fig);
plot(tGrid, V_tilde_grid);
title('V tilde');
figCounter = figCounter + 1;

fig = figure(figCounter);
clf(fig);
plot(tGrid, V_tilde_dot_grid);
title('V tilde dot');
figCounter = figCounter + 1;


%% Plot what would happen for different u's around contact time.
% allows us to compare deadzone and other stuff

figCounter = 11;

disp('------------')
tBreaks = xtraj.getBreaks();
t = 1.401
[~,idx] = min(abs(t-tBreaks));
t = tBreaks(idx)
% t = t_contact - 0.01;
S = hzdController.S;

y = controlTrajs.controlData.y.eval(t);
ydot = controlTrajs.controlData.ydot.eval(t);
A_y = controlTrajs.controlData.A_y.eval(t);
B_y = controlTrajs.controlData.B_y.eval(t);

V_actual = controlTrajs.controlData.V.eval(t);
V_dot_actual = controlTrajs.controlData.V_dot.eval(t);

yTilde = controlTrajs.controlDataOther.y.eval(t);
ydotTilde = controlTrajs.controlDataOther.ydot.eval(t);
A_y_Tilde = controlTrajs.controlDataOther.A_y.eval(t);
B_y_Tilde = controlTrajs.controlDataOther.B_y.eval(t);


x = xtraj.eval(t);
uActual = controlTrajs.controlData.u.eval(t);

uGrid = linspace(-2,2);
onesGrid = ones(size(uGrid));


V_dot = 2*[y, ydot] * S * [ydot*onesGrid; A_y + B_y*uGrid];

V_dot_test = 2*[y, ydot] * S * [ydot; A_y + B_y*uActual]
V_dot_actual

% increment the figure counter
% figCounter = figCounter + 1;

%%

t_contact = xtraj.te(2);
plotVdot(hzdController, controlTrajs, t_contact);


%% More testing

fig = figure(21);
clf(fig);
handle = {}
S = hzdController.S;

t = 1.401
[~,idx] = min(abs(t-tBreaks));
t = tBreaks(idx)

y = controlTrajs.controlData.y.eval(t);
ydot = controlTrajs.controlData.ydot.eval(t);
A_y = controlTrajs.controlData.A_y.eval(t);
B_y = controlTrajs.controlData.B_y.eval(t);

V_actual = controlTrajs.controlData.V.eval(t);
V_dot_actual = controlTrajs.controlData.V_dot.eval(t);

yTilde = controlTrajs.controlDataOther.y.eval(t);
ydotTilde = controlTrajs.controlDataOther.ydot.eval(t);
A_y_Tilde = controlTrajs.controlDataOther.A_y.eval(t);
B_y_Tilde = controlTrajs.controlDataOther.B_y.eval(t);

uActual = controlTrajs.controlData.u.eval(t);

uGrid = linspace(-2,2);
onesGrid = ones(size(uGrid));


V_dot = [y, ydot] * S * [ydot*onesGrid; A_y + B_y*uGrid];

% V_dot_test = 2*[y, ydot] * S * [ydot; A_y + B_y*uActual]
% V_dot_actual

% need a minus here because the control input is synced with the mode and
% switches sign during mode changes

V_dot_Tilde = [yTilde, ydotTilde] * S * [ydotTilde*onesGrid; A_y_Tilde - B_y_Tilde*uGrid];


%     clf(fig);

for i=1:length(handle)
    delete(handle{i});
end

hold on;
handle{1} = plot(uGrid, V_dot, 'b', 'DisplayName', 'V dot');
handle{2} = plot(uGrid, V_dot_Tilde, 'r', 'DisplayName', 'V dot Tilde');
handle{3} = scatter(uActual, V_dot_actual, 150, 'g', 'filled', 'DisplayName', 'V dot actual');
% legend('show');
titleString = strcat('t = ', num2str(t));
title(titleString);
hold off;


































