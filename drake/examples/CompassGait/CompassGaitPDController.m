classdef CompassGaitPDController < handle


  properties
    xtraj_;
    utraj_;
    xtrajPhase_;
    xtrajDot_;
    utrajPhase_;
    xtrajDotPhase_;
    plant_;
  end

  methods
    function obj = CompassGaitPDController(plant, xtraj, utraj, options)
      obj.xtraj_ = xtraj;
      obj.utraj_ = utraj;
      obj.plant_ = plant;
    end

    function initializeOptions(obj, options)

      defaultOptions = struct();
      defaultOptions.Kp = 100;
      defaultOptions.dampingRatio = 0.7;

      options = applyDefaults(options, defaultOptions);
      obj.options_ = options;
    end

    % this is basically just copied from HZDController
    function generatePhaseTrajectory(obj)
      tBreaks = x.getBreaks();
      xDeriv = x.fnder(1);

      xTrueGrid = x.eval(tBreaks);
      xDerivTrueGrid = xDeriv.eval(tBreaks);

      hDerivTrueGrid = xTrueGrid(3,:)./xTrueGrid(4,:);

      hDeriv2TrueGrid = xDerivTrueGrid(3,:)./xTrueGrid(4,:).^2 - ...
        hDerivTrueGrid.*(xDerivTrueGrid(4,:))./xTrueGrid(4,:).^2;

      uTrueGrid = u.eval(tBreaks);

      phaseGrid = xGrid(2,:);
      obj.hdTraj = PPTrajectory(pchip(phaseGrid, xGrid(1,:)));
      obj.hdTraj_deriv = PPTrajectory(pchip(phaseGrid, hDerivTrueGrid));
      ob.hdTraj_dderiv = PPTrajectory(pchip(phaseGrid, hDeriv2TrueGrid));
      obj.utrajPhase_ = PPTrajectory(pchip(phaseGrid, uTrueGrid));
      obj.xtrajPhase_ = PPTrajectory(pchip(phaseGrid, xTrueGrid));
      obj.xtrajDotPhase_ = PPTrajectory(pchip(phaseGrid, xDerivTrueGrid));

      obj.xtrajDot_ = PPTrajectory(pchip(tBreaks, xDerivTrueGrid));
    end

    function data = getSwingLegPlan(obj, phaseVar)
      data = struct();
      x_plan = obj.xtrajPhase_.eval(phaseVar);
      x_deriv_plan = obj.xtrajDotPhase_.eval(phaseVar);

      data.pos = x_plan(1);
      data.vel = x_plan(3);
      data.acc = x_deriv_plan(3);
    end
  end

end