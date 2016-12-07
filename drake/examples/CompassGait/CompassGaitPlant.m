classdef CompassGaitPlant < HybridDrakeSystem
  
  properties
    m = 5;
    mh = 10;
    a = 0.5;
    b = 0.5;
    g = 9.8;
    l;
    gamma;
    useFixedOutputCoords = false;
  end
  
  methods 
    function obj = CompassGaitPlant(gammaIn)
      
      


      obj = obj@HybridDrakeSystem(1,7);
      % allow passing in of ground slope
      if (nargin > 0)
        obj.gamma = gammaIn;
      else
        obj.gamma = 3*pi/180;
      end
      obj.l=obj.a+obj.b;
      p = CompassGaitStancePlant(obj.m,obj.mh,obj.a,obj.b,obj.l,obj.g);
      obj = setInputFrame(obj,p.getInputFrame);
      outputFrame = CoordinateFrame('CompassGaitPlantOutput', 7, 'x', {'mode','theta1','theta2','theta1_dot','theta2_dot','u', 'plan time'});
      obj = obj.setOutputFrame(outputFrame);
%       obj = setOutputFrame(obj,p.getOutputFrame);
      
      obj = obj.addMode(p);
      obj = obj.addMode(p);

      obj = addTransition(obj,1,andGuards(obj,@footCollisionGuard1,@footCollisionGuard2),@collisionDynamics,false,true);
      obj = addTransition(obj,2,andGuards(obj,@footCollisionGuard1,@footCollisionGuard2),@collisionDynamics,false,true);

      % obj = obj.setOutputFrame(obj.getStateFrame)
      %      obj.ode_options = odeset('InitialStep',1e-3, 'Refine',1,'MaxStep',0.02);
      obj = setSimulinkParam(obj,'InitialStep','1e-3','MaxStep','0.05');
      obj = setInputLimits(obj,-50,50);
    end
    
    function [g,dg] = footCollisionGuard1(obj,t,x,u)
      g = x(1)+x(2)+2*obj.gamma;  % theta_st - gamma <= - (theta_sw - gamma) 
      dg = [0,1,1,0,0,0];
    end
    
    function [g,dg] = footCollisionGuard2(obj,t,x,u);
      g = -x(1);   % theta_st >= 0
      dg = [0,-1,0,0,0,0];
    end


    function toeHeight = computeToeHeight(obj, x)

      if (size(x) == 4)
        thetaSwing = x(1);
        thetaStance = x(2);
      else
        thetaSwing = x(2);
        thetaStance = x(3);
      end

      toeHeightRelativeToStanceFoot = obj.l*(cos(thetaStance) - cos(thetaSwing));

      % positive is more to the right
      xDisplacementRelativeToStanceFoot = obj.l*(-sin(thetaStance) + sin(thetaSwing));

      % how much has the ground height changed relative to where stance foot is
      groundSlope = -tan(obj.gamma);
      groundHeightChangeFromStanceFoot = groundSlope * xDisplacementRelativeToStanceFoot;

      % this is the height of the toe above (or below if negative) the ground
      toeHeight = toeHeightRelativeToStanceFoot - groundHeightChangeFromStanceFoot;
    end
      
    function [xp,mode,status,dxp] = collisionDynamics(obj,mode,t,xm,u)
      m=obj.m; mh=obj.mh; a=obj.a; b=obj.b; l=obj.l;
      
      if (mode==1) mode=2;  % switch modes
      else mode=1; end
      
      alpha = (xm(1) - xm(2));
      Qm = [ -m*a*b, -m*a*b + (mh*l^2 + 2*m*a*l)*cos(alpha); 0, -m*a*b ];
      Qp = [ m*b*(b - l*cos(alpha)), m*l*(l-b*cos(alpha)) + m*a^2 + mh*l^2; m*b^2, -m*b*l*cos(alpha) ];
      
      xp = [xm([2,1]); ...     % switch stance and swing legs
        Qp\Qm*xm(3:4)];   % inelastic impulse
      
      if (nargout>3)
        Qpi = inv(Qp);
        dalpha = [1 -1];  % d/dq
        dQpdalpha = [m*b*l*sin(alpha), m*l*b*sin(alpha); 0, m*b*l*sin(alpha) ];
        dQpidalpha = -Qpi*dQpdalpha*Qpi;
        dQmdalpha = [0, -(mh*l^2 + 2*m*a*l)*sin(alpha); 0, 0];
        dxpdxm = zeros(4);
        dxpdxm(1:2,1:2) = [ 0 1 ; 1 0];
        dxpdxm(3:4,3:4) = Qpi*Qm;
        dxpdxm(3:4,1) = (dQpidalpha*dalpha(1))*Qm*xm(3:4) + Qpi*(dQmdalpha*dalpha(1))*xm(3:4);
        dxpdxm(3:4,2) = (dQpidalpha*dalpha(2))*Qm*xm(3:4) + Qpi*(dQmdalpha*dalpha(2))*xm(3:4);
        dxp = [zeros(4,2),dxpdxm,zeros(4,1)];
      end
      status = 0;
    end


    function y = output(obj,t,x,u)
      y = [x;u;0]; % just put in a place holder for plan time

      % switch the order if that is specified
      if (obj.useFixedOutputCoords)
        swingLegIdx = 2;
        stanceLegIdx = 3;

        if (x(1)==2)
          swingLegIdx = 3;
          stanceLegIdx = 2;
        end

        y(swingLegIdx) = x(2);
        y(stanceLegIdx) = x(3);
        y(swingLegIdx+2) = x(4);
        y(stanceLegIdx+2) = x(5);
        
      end        
    end

    % the output of this will be 4 x 4
    function dxpdxm = getResetMapJacobian(obj, x)
      mode = 0;
      t = 0;
      u = 0;
      [xp, mode, status, dxp] = obj.collisionDynamics(mode, t, x, u);

      dxpdxm = dxp(:,3:6); % should be 4 x 4
    end
    
  end
  
  methods (Static)
    function run()
      r = CompassGaitPlant();
      v = CompassGaitVisualizer(r);
      traj = simulate(r,[0 10]);%,[2;0; 0; 2.0; -0.4]);
      playback(v,traj);    
    end
  end  
  
end
