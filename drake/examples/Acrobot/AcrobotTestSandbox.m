
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

%% Setup mixed integer simulator
dt = p.timestep;
miStep = MIStep(p.getManipulator(),dt);

%% Try to simulate one step
miStep.step(0,x0,0);