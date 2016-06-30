
% Script for simulating the Acrobot with a floating base 

options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('AcrobotNoBase.urdf',.01,options);
warning(w);
v = p.constructVisualizer(struct('viewer','BotVisualizer'));

%% Setup the initial state
dTheta_1 = 0.0;
dTheta_2 = 0.05;
x0 = zeros(p.getNumStates(),1);
x0(2) = 3; % z height
x0(3) = pi + dTheta_1; % angle of base/shoulder link
x0(4) = dTheta_2; % ankle of elbow link
x0 = p.resolveConstraints(x0);
v.drawWrapper(0,x0);

% vertical one meter above the ground
qNominal = zeros(p.getNumPositions, 1);
qNominal(2) = 1;
qNominal(3) = pi;

vZero = zeros(p.getNumVelocities,1);




%% Construct a controller and simulate with misim
controllerOptions = struct();

% activate early
if (false)
  controllerOptions.activateEarly = true;
  controllerOptions.activateEarlyPhi = 10;
end


% activate late
if (false)
  controllerOptions.activateLate = true;
  controllerOptions.activateLateTimeDelay = 0.1;
end

% constant control input
if (false)
  controllerOptions.setConstantControlInput = true;
  controllerOptions.constantControlInput = 0.0;
end

c = AcrobotController(p, controllerOptions);
dt = 0.01;
T = 5;
N = ceil(T/dt);
mu = 2;
[xtraj,misimOutput] = misim(getManipulator(p),x0,dt,N,mu,c);
v.playback(xtraj, struct('slider',true));
alphaTraj = misimOutput.alphaTraj;
uTraj = misimOutput.uTraj;
v.playback(xtraj, struct('slider',true));

% %% Test drawing
% xt = zeros(p.getNumStates(),1);
% xt(3) = pi/2;
% v.drawWrapper(0,xt);
% 
% %% Standard Acrobot
% r = RigidBodyManipulator('Acrobot.urdf');
% vA = r.constructVisualizer(struct('viewer','BotVisualizer'));
% xA = zeros(4,1);
% xA(1) = pi/2;
% vA.drawWrapper(0,xA);


% %% Simulate with DrakeSystem simulate call
% xtraj_LCP = simulate(p,[0,T],x0);
% xtraj_LCP = xtraj_LCP.setOutputFrame(p.getStateFrame);
% v.playback(xtraj_LCP, struct('slider',true));

%% Testing
if false
  disp('testing')
  t = .1
  xtraj.eval(t)
  useStanceController = c.useStanceController(t,xtraj.eval(t))
  u = c.output(t,0,xtraj.eval(t))
  alphaVal = alphaTraj.eval(t);
  % alphaVal(misimOutput.binary_normal_inds)
end

%% Trajectory information
fprintf('first contact time = %d \n', c.firstContactTime);
fprintf('stance controller activation time = %d \n', c.stanceControllerActivationTime);
activationDelta = c.stanceControllerActivationTime - c.firstContactTime;
fprintf('stance controller activation time delta = %d \n', activationDelta);




%% Visualization
tspan = xtraj.tspan;
tPlot = [0.1, 0.3];
tPlot = tspan;
tGrid = tPlot(1):dt:tPlot(2);
alphaGrid = alphaTraj.eval(tGrid);
xGrid = xtraj.eval(tGrid);

theta1_idx = 3;
theta2_idx = 4;
theta1_dot_idx = 3+4;
theta2_dot_idx = 4+4;

binaryNormal = alphaGrid(misimOutput.binary_normal_inds, :);
binaryPosSlide = alphaGrid(misimOutput.binary_pos_slide_inds, :);
binaryNegSlide = alphaGrid(misimOutput.binary_neg_slide_inds,:);
normalForce = alphaGrid(misimOutput.normal_force_inds, :);
frictionForce = alphaGrid(misimOutput.friction_force_inds,:);
controlInput = uTraj.eval(tGrid);

fig = figure(1);
clf(fig);
hold on
plot(tGrid, binaryNormal, 'b')
% plot(tGrid, binaryPosSlide, 'r')
% plot(tGrid, binaryNegSlide, 'g')
legend('binaryNormal')
hold off

fig = figure(2);
clf(fig);
hold on
plot(tGrid, normalForce, 'b')
plot(tGrid, frictionForce, 'r')
legend('normal force', 'friction force')
hold off

fig = figure(3);
clf(fig);
hold on
plot(tGrid, controlInput,'b')
plot(tGrid, binaryNormal, 'r')

legend('control input', 'binary normal')
hold off

fig = figure(4);
clf(fig)
hold on
plot(tGrid, xGrid(theta1_idx,:), 'b');
plot(tGrid, xGrid(theta1_dot_idx,:),'b--');
plot(tGrid, xGrid(theta2_idx,:), 'r');
plot(tGrid, xGrid(theta2_dot_idx,:), 'r--');
plot(tGrid, binaryNormal, 'g')
legend('theta1', 'theta1 dot', 'theta2', 'theta2 dot', 'binary normal');
hold off

%% Sandbox
[H,C,B] = p.manipulatorDynamics(qNominal,vZero);
[H_t,C_t,B_t] = p.manipulatorDynamics(qNominal,vZero);

%% Flight phase LQR controller
% Q_flight = 0*eye(8);
% Q_flight(3,3) = 10;
% Q_flight(3+4,3+4) = 1;
% 
% R_flight = 1;
% 
% tilqr(p,xNominal, 0,Q_flight, R_flight)