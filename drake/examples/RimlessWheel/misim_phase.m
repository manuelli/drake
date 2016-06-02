function misim

options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('RimlessWheel.urdf',.01,options);
warning(w);

figure(1); clf;  hold on;
for thetadot0 = 0:0.1:1
  x0 = [0;1;0;thetadot0;0;thetadot0];
  xtraj = misim(getManipulator(p),x0,.02,140);
  fnplt(xtraj,[3,6]);
end
  
%v = p.constructVisualizer();
%v.axis = [-2.5 2.5 -.1 3];
%v.playback(xtraj);
