classdef AcrobotController < handle

  properties(Access=public)
    plant;
    acrobotPlant;
    nq;
    balanceLQRController;
    balanceLQRControllerStruct;
    u_max = 20;
    firstContactTime = -1; % placeholder
    stanceControllerActivationTime = -1;
    stanceControllerActive = false;
    options;
  end

  methods
    function obj = AcrobotController(plant, options)
      if nargin < 2
        options = struct();
      end

      options = applyDefaults(options, struct('activateEarly', false, 'activateEarlyPhi', 0.05, 'activateLate', false, 'activateLateTimeDelay', 0.05,...
        'setConstantControlInput', false, 'constantControlInput', 0))

      if options.activateEarly && options.activateLate
        error('can only specify one of early or late activation of stance controller')
      end
      obj.options = options;
      obj.plant = plant;
      obj.nq = obj.plant.getNumPositions();
      obj.acrobotPlant = AcrobotPlant(); % make sure we set the params to match those in the plant
      obj = obj.constructLQRBalancingController();
      
    end

    function obj = constructLQRBalancingController(obj)
      Q = diag([10,10,1,1]); R = 1;
      x0 = obj.acrobotPlant.xG;
      u0 = obj.acrobotPlant.uG;
      obj.balanceLQRController = tilqr(obj.acrobotPlant,x0,u0,Q,R);
      obj.balanceLQRControllerStruct.x0 = [pi;0;0;0];
      obj.balanceLQRControllerStruct.u0 = 0;
    end

    function y = output(obj,t,~,x)
      y = 0;

      % by default do nothing
      if (obj.options.setConstantControlInput)
        y = obj.options.constantControlInput;
        return;
      end
      

      if(~obj.useStanceController(t,x))
        y
        return
      end

      % if we are in contact then do the lqrBalancing controller
      uLQR = obj.getLQRBalanceControlInput(t,x);
      uLQR = min(uLQR,obj.u_max);
      uLQR = max(uLQR,-obj.u_max);
      y = uLQR;
      y
    end

    function u = getLQRBalanceControlInput(obj,t,x)
      % need to be really careful with transforming frames and stuff here
      xAcrobot = AcrobotController.convertStateToStandardAcrobotState(x);
      xErr = xAcrobot - obj.balanceLQRControllerStruct.x0;
      u = obj.balanceLQRController.output(t,0,xErr) + obj.balanceLQRControllerStruct.u0;
    end

    function [y, phiC] = isInContact(obj,x)
      q = x(1:obj.nq);
      phiC = obj.plant.contactConstraints(q,false);
      phiC

      % check to see whether we are in contact or not
      y = false;
      if (phiC < 1e-4)
        y = true;
      end
    end


    % applies the logic to decide whether or not we should use the stance controller
    % allows for activating early, late or right on time
    function val = useStanceController(obj,t,x)
      val = false;
      [y,phiC] = obj.isInContact(x);
      % once the stance controller is activated, just keep it on
      % this is a hack but works for now
      if obj.stanceControllerActive
        val = true;
        return;
      end
      

      if ((obj.firstContactTime < 0) && y)
        obj.firstContactTime = t;
      end

      if obj.options.activateEarly
        if (phiC < obj.options.activateEarlyPhi)
          val = true;
        end
      elseif obj.options.activateLate
        if (obj.firstContactTime > 0) && (obj.options.activateLateTimeDelay < (t - obj.firstContactTime))
          val = true;
        end
      else
        % if we reach here then neither early/late option was specified for the controller so just do the usual thing, engage if in contact
        val = y;
      end
        

      if val && (obj.stanceControllerActivationTime < 0)
        obj.stanceControllerActivationTime = t;
      end

      % activate the stance controller if it hasn't already happened
      if val
        obj.stanceControllerActive = true;
      end
    end

  end
  methods(Static)
    function xAcrobot = convertStateToStandardAcrobotState(x)
      numPositions = 4;
      xAcrobot = zeros(4,1);
      xAcrobot(1) = x(3); % Floating base angle state corresponds to angle of upper link
      xAcrobot(2) = x(4); % Elbow angle is same, just with different idx
      xAcrobot(3) = x(3+numPositions); % Floating base angle velocity corresponds to upper link angle dot
      xAcrobot(4) = x(4+numPositions);
    end
  end


end