
% Script for simulating the Acrobot with a floating base 

options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('AcrobotNoBase.urdf',.01,options);
warning(w);
v = p.constructVisualizer(struct('viewer','BotVisualizer'));

%%
x0 = zeros(p.getNumStates(),1);
x0(3) = pi;
x0(2) = 0.5;
x0(4) = 0.01;
v.drawWrapper(0,x0);

%% simulate with misim
dt = 0.02;
T = 2;
N = ceil(T/dt);
xtraj = misim(getManipulator(p),x0,dt,N);
v.playback(xtraj, struct('slider',true));
