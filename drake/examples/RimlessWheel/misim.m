function misim

options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('RimlessWheel.urdf',.01,options);
warning(w);

%options.visualize = true;
%x0 = [0;cos(pi/8);pi/8;zeros(3,1)];
%x0 = p.findFixedPoint(randn(6,1),[],options);

x0 = p.resolveConstraints([0;1+rand;randn;5*rand;randn;5*rand]);
xtraj = misim(getManipulator(p),x0,.02,200);

v = p.constructVisualizer();
v.axis = [-2.5 2.5 -.1 3];
v.playback(xtraj);

return;

[H,C,B] = p.manipulatorDynamics(x0(1:3),x0(4:6));

[phiC,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = p.contactConstraints(x0);
J = [n;cell2mat(D)]

% draw vector field
[theta,thetadot] = meshgrid(linspace(0,pi/4,22),1.5*linspace(-1,1,21));
for i=1:numel(theta)
  if (theta(i)>=pi/8)
    x(:,i) = [0;
      cos(pi/4-theta(i));
      theta(i);
      thetadot(i)*cos(pi/4-theta(i));  %note: playing some tricks w/ the coordinate system to make the math simpler here
      thetadot(i)*sin(pi/4-theta(i));
      thetadot(i)];
  else    
    x(:,i) = [0;cos(theta(i));theta(i);thetadot(i)*cos(theta(i));-thetadot(i)*sin(theta(i));thetadot(i)];
  end
  xn(:,i) = p.update(0,x(:,i),[]);
end

figure(1); clf;
quiver(x(3,:),x(6,:),xn(3,:)-x(3,:),xn(6,:)-x(6,:))%,1.5);
axis tight;