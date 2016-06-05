function misim

options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('CompassGait.urdf',.01,options);
warning(w);

%options.visualize = true;
%x0 = [0;cos(pi/8);pi/8;zeros(3,1)];
%x0 = p.findFixedPoint(randn(6,1),[],options);

v = p.constructVisualizer();
v.axis = [-2.5 2.5 -.1 3];

x0 = .1*randn(getNumStates(p),1);  x0(4) = .2;
x0 = p.resolveConstraints(x0);
xtraj = misim(getManipulator(p),x0,.02,200);

v.playback(xtraj);

