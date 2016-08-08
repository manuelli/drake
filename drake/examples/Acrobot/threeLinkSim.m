% Script for simulating the Acrobot with a floating base 

options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('3Link.urdf',.01,options);
warning(w);
v = p.constructVisualizer(struct('viewer','BotVisualizer'));
r = p.getManipulator();
numPositions = r.getNumPositions();
numVelocities = r.getNumVelocities();

%% Visualize an initial state
q0 = [0;4.5;0.0;0.7;-1.3];
v0 = 0*q0;
x0 = [q0; v0];
v.drawWrapper(0,x0)

%% Get H,C,B matrices and stuff
kinsol = r.doKinematics(x0);
[H,C,B] = r.manipulatorDynamics(q0,v0);

% footPos = [0;0];
footPos = [0;-2]
lowerLegId = r.findLinkId('lower_leg_link');
% should be standard jacobian to the toe
[pt, J] = r.forwardKin(kinsol, lowerLegId, footPos);

%% Figure out dynamics

D = inv(H)*(J')
E = inv(H)*B

%% Draw some debugging stuff
lcmgl = LCMGLClient();
posInWorld = [pt(1);0;pt(2)];

lcmgl.glColor3f(1,1,0)
lcmgl.sphere(posInWorld, 0.04,20,20);
lcmgl.switchBuffers()

%% Debugging
