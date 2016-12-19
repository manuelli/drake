fig = figure(6);
clf(fig);
hold on;
fnplt(xtraj_single_step, [1,2])
fnplt(xtraj_single_step, [2,1])
hold off;

%%
t = 0.84;
idx = floor((t-t_start)/dt)
t_actual = tArray(idx)

kfParticle = kalmanFilterParticleArray{idx};
kfParticle.x_
kfParticle.sigma_
disp('hybrid mode')
kfParticle.hybridMode_
options = struct();
options.sigmaGlobal = kfParticle.sigma_;
particleFilter.initializeFilter(kfParticle.hybridMode_, kfParticle.x_,options)

xLocal = cgUtils.transformGlobalStateToLocalState(kfParticle.hybridMode_, kfParticle.x_)
guardVal = plant.eventFun(0,xLocal)

fig = figure(1);
clf(fig);
hold on;
ekf.plotKalmanFilterState(kfParticle, struct('plotType','position'));

particleFilter.plotParticleSet(particleFilter.particleSet_, fig, struct('plotType','position', 'nominalPlotType','position'))

%%

uTraj = PPTrajectory(pchip(tArray, uArray));

fig = figure(15);
clf(fig);
plot(tArray, uArray)

%%
figHandle_ = figure(31);
plotData.figHandle_ = figHandle_;


plotUtils = PlotUtility(plotData);

idx = 103;
tArray(idx)
particleSet = plotData.particleSetArray{idx};
uGlobal = plotData.uArray(idx);

% uGlobal = 0;
plotUtils.setupPlots()

%% HJB cost function test
disp('-------------')
t = 0.82;
timeIdxTraj = foh(tArray, 1:length(tArray));
idx = floor(ppval(timeIdxTraj, t))
t_actual = tArray(idx)

uActual = uArray(idx);

t = 0.815
xLocal = xtraj_opt.eval(t);
xGlobal = cgUtils.transformLocalStateToGlobalState(1, xLocal);
trueParticleData = struct();
trueParticleData.hybridMode = 1;
trueParticleData.xGlobal = xGlobal;
trueParticle = CompassGaitParticle(trueParticleData);

% trueParticle = trueParticleArray{idx};
ekfParticle = kalmanFilterParticleArray{idx};

varMultiplier = 0.3;

testOptions = struct();
testOptions.sigmaGlobal = varMultiplier*ekfParticle.sigma_;
particleFilter.initializeFilter(trueParticle.hybridMode_, trueParticle.x_, testOptions);
% particleSet = particleFilter.particleSet_;


xLocal = cgUtils.transformGlobalStateToLocalState(trueParticle.hybridMode_, trueParticle.x_);
[xPlus, newMode] = plant.collisionDynamics(trueParticle.hybridMode_,0, xLocal, 0);
newParticleData = struct();
newParticleData.hybridMode = newMode;
newParticleData.xGlobal = cgUtils.transformLocalStateToGlobalState(newMode, xPlus);
resetMapTrueParticle = CompassGaitParticle(newParticleData);
particleSet = {trueParticle, resetMapTrueParticle};



figCounter = 23;
fig = figure(figCounter);
clf(fig);
subplot(2,1,1);
hold on;
particleFilter.plotNominalTraj()
particleFilter.plotSingleParticle(trueParticle, struct('colorString', 'g'));
particleFilter.plotParticleSet(particleSet, fig);
ekf.plotKalmanFilterState(ekfParticle);
hold off;

subplot(2,1,2);
plotOptions = struct();
plotOptions.plotType = 'right';

trueParticleOptions = plotOptions;
trueParticleOptions.colorString = 'g';
hold on;
particleFilter.plotNominalTraj()
particleFilter.plotSingleParticle(trueParticle, trueParticleOptions);
particleFilter.plotParticleSet(particleSet, fig, plotOptions);
ekf.plotKalmanFilterState(ekfParticle, plotOptions);
hold off;

uRobust = hzdController.computeRobustControlFromParticleSet(particleSet, 0.05)
% uActual
% uActual_alt = hzdController.getControlInputFromGlobalState(t_actual, trueParticle.hybridMode_, trueParticle.x_)


figCounter = figCounter + 1;

fig = figure(figCounter);
clf(fig);
% plotUtils.plotStandardControlInputForEachParticleInParticleSet(particleSet);

plot_dt = 0.005;
tBreaks = 0.78:plot_dt:0.819;
plotUtils.plotControlOnBothSidesOfResetMap(tBreaks, xtraj_opt);

%% Test some HZD controller stuff
theta = 0.02
x = hzdController.xPhaseTraj.eval(theta)
thetaDot = x(4);

data = hzdController.computeHybridZeroDynamics([1;x]);

data.A_y
data.A_y_theta
data.A_y_H


%% Full state LQR
Q = 1*diag([2,2,1,1]);
R = 1;
cgLQR = CompassGaitLQR(plant, xtraj_opt, utraj_opt);
[tGrid, S_traj, K_traj] = cgLQR.computeLQRValueFunction(Q,R);


fig = figure(55);
clf(fig);
subplot(2,1,1);
hold on;
plot(tGrid, K_traj(:,1), 'b', 'DisplayName','swing');
plot(tGrid, K_traj(:,2), 'r', 'DisplayName','stance');
legend('show');
title('position gains');
hold off;

subplot(2,1,2);
hold on;
plot(tGrid, K_traj(:,3), 'b', 'DisplayName','swing');
plot(tGrid, K_traj(:,4), 'r', 'DisplayName','stance');
title('velocity gains');
legend('show');
hold off;

K_traj(end,1)
K_traj(end,2)




%% Test lqr stuff
disp('----------------')
A = zeros(2,2);
A(1,2) = 1;
B = [0;1];

Q = diag([10,2]);
R = 1;

[K,S] = lqr(A,B,Q,R);
K
dampingRatio = K(2)/(sqrt(K(1))*2)

%%

tspan = xtraj_opt.tspan;
t_f = tspan(2);
eps = 0.01;
xstar = xtraj_opt.eval(t_f);
dx = [0.005;0;0;0];

xm = xstar + dx;


T = cgLQR.getResetMapJacobian(xstar);


xPlus = plant.collisionDynamics(0,0,xstar,0);
xPlusTraj = xtraj_opt.eval(0);

xPlusActual = plant.collisionDynamics(0,0,xm,0)
xPlusEst = xPlus + T*dx

diff = xPlusActual - xPlusEst


%%

disp('-------------')
t = 1.2;
timeIdxTraj = foh(tArray, 1:length(tArray));
idx = floor(ppval(timeIdxTraj, t));
t = tArray(idx);

tp = trueParticleArray{idx};
ekfParticle = kalmanFilterParticleArray{idx};

disp('------');
[uGlobal, data] = cgLQR.getControlInputFromGlobalState(tp.hybridMode_, tp.x_)

uArray(idx)

[uGlobal_ekf, data_ekf] = cgLQR.getControlInputFromGlobalState(ekfParticle.hybridMode_, ekfParticle.x_)







































