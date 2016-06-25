
% Script for simulating the Acrobot with a floating base 

options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('AcrobotNoBase.urdf',.01,options);
warning(w);
v = p.constructVisualizer(struct('viewer','BotVisualizer'));

%% Serup the initial state
x0 = zeros(p.getNumStates(),1);
x0(3) = pi;
x0(2) = 0.5;
x0(4) = 0.01;
v.drawWrapper(0,x0);

%% Construct a controller
c = AcrobotController(p);

%% simulate with misim
dt = 0.02;
T = 2;
N = ceil(T/dt);
[xtraj,misimOutput] = misim(getManipulator(p),x0,dt,N);
v.playback(xtraj, struct('slider',true));
alphaTraj = misimOutput.alphaTraj;

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

%% Testing
t = 0.6
c.isInContact(xtraj.eval(t))
alphaVal = alphaTraj.eval(t);
alphaVal(misimOutput.binary_normal_inds)