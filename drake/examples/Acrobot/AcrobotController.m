classdef AcrobotController

  properties
    plant;
    acrobotPlant;
    nq;
    balanceLQRController;
  end

  methods
    function obj = AcrobotController(plant, options)
      if nargin < 2
        options = struct();
      end
      obj.plant = plant;
      obj.nq = obj.plant.getNumPositions();
      obj.acrobotPlant = AcrobotPlant();
      obj = obj.constructLQRBalancingController();
    end

    function obj = constructLQRBalancingController(obj)
      Q = diag([10,10,1,1]); R = 1;
      x0 = obj.acrobotPlant.xG;
      u0 = obj.acrobotPlant.uG;
      obj.balanceLQRController = tilqr(obj.acrobotPlant,x0,u0,Q,R);
    end

    function y = output(t,~,x)
      % by default do nothing
      y = 0;

      if(~obj.isInContact(x))
        return
      end

      % if we are in contact then do the lqrBalancing controller
    end

    function u = getLQRBalanceControlInput(t,x)
      xAcrobot = AcrobotController.convertStateToStandardAcrobotState(x);
      u = obj.balanceLQRController.output(t,0,xAcrobot);
    end

    function y = isInContact(obj,x)
      q = x(1:obj.nq);
      phiC = obj.plant.contactConstraints(q,false)

      % check to see whether we are in contact or not
      y = false;
      if (phiC < 1e-4)
        y = true;
      end
    end

  end
  methods(Static)
    function xAcrobot = convertStateToStandardAcrobotState(obj, x)
      numPositions = 4;
      xAcrobot = zeros(4,1);
      xAcrobot(1) = x(3); % Floating base angle state corresponds to angle of upper link
      xAcrobot(2) = x(4); % Elbow angle is same, just with different idx
      xAcrobot(3) = x(3+numPositions); % Floating base angle velocity corresponds to upper link angle dot
      xAcrobot(4) = (4+numPositions);
    end
  end


end