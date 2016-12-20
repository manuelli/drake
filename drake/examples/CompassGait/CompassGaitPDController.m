classdef CompassGaitPDController < handle


  properties
    xtraj_;
    utraj_;
    xtrajPhase_;
    xtrajDot_;
    utrajPhase_;
    xtrajDotPhase_;
    plant_;
    cgUtils_;
    Kp_;
    Kd_;
    options_; 
  end

  methods
    function obj = CompassGaitPDController(plant, xtraj, utraj, options)
      if nargin < 4
        options = struct();
      end
      obj.xtraj_ = xtraj;
      obj.utraj_ = utraj;
      obj.plant_ = plant;
      obj.cgUtils_ = CompassGaitUtils();

      obj.initializeOptions(options);
      obj.generatePhaseTrajectory();
    end

    function initializeOptions(obj, options)

      defaultOptions = struct();
      defaultOptions.Kp = 50;
      defaultOptions.dampingRatio = 0.7;

      options = applyDefaults(options, defaultOptions);
      obj.options_ = options;

      obj.Kp_ = options.Kp;
      obj.Kd_ = 2*options.dampingRatio*sqrt(options.Kp);
    end

    % this is basically just copied from HZDController
    function generatePhaseTrajectory(obj)
      tBreaks = obj.xtraj_.getBreaks();
      xDeriv = obj.xtraj_.fnder(1);

      xTrueGrid = obj.xtraj_.eval(tBreaks);
      xDerivTrueGrid = xDeriv.eval(tBreaks);

      hDerivTrueGrid = xTrueGrid(3,:)./xTrueGrid(4,:);

      hDeriv2TrueGrid = xDerivTrueGrid(3,:)./xTrueGrid(4,:).^2 - ...
        hDerivTrueGrid.*(xDerivTrueGrid(4,:))./xTrueGrid(4,:).^2;

      uTrueGrid = obj.utraj_.eval(tBreaks);

      phaseGrid = xTrueGrid(2,:);
      % obj.hdTraj = PPTrajectory(pchip(phaseGrid, xTrueGrid(1,:)));
      % obj.hdTraj_deriv = PPTrajectory(pchip(phaseGrid, hDerivTrueGrid));
      % ob.hdTraj_dderiv = PPTrajectory(pchip(phaseGrid, hDeriv2TrueGrid));
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

    function data = computeDynamicsData(obj, xLocal)
      q = xLocal(1:2);
      qdot = xLocal(3:4);
      [H,C,B] = obj.plant_.modes{1}.manipulatorDynamics(q,qdot);

      data = struct();
      data.constantTerm = -inv(H)*C;
      data.linearTerm = inv(H)*B;
    end

    function [u, controlData] = getControlInput(obj, xLocal)
      dynamicsData = obj.computeDynamicsData(xLocal);

      swingPos = xLocal(1);
      swingVel = xLocal(3);

      phaseVar = xLocal(2); % stance leg angle
      planData = obj.getSwingLegPlan(phaseVar);

      swingAcc_ff = planData.acc;
      swingAcc_fb = -obj.Kp_*(swingPos - planData.pos) - obj.Kd_*(swingVel - planData.vel);


      swingAcc_des = swingAcc_ff + swingAcc_fb;

      % swingAcc = dynamicsData.constantTerm + dynamicsData.linearTerm*u
      u = (swingAcc_des - dynamicsData.constantTerm(1))/dynamicsData.linearTerm(1);

      controlData = dynamicsData;
      controlData.phaseVar = phaseVar;
      controlData.swingAcc_ff = swingAcc_ff;
      controlData.swingAcc_fb = swingAcc_fb;
      controlData.swingAcc = swingAcc_des;
    end

    function [uGlobal, controlData] = getControlInputFromGlobalState(obj, hybridMode, xGlobal)
      xLocal = obj.cgUtils_.transformGlobalStateToLocalState(hybridMode, xGlobal);

      [uLocal, controlData] = obj.getControlInput(xLocal);

      uGlobal = obj.cgUtils_.transformLocalControlToGlobalControl(hybridMode, uLocal);

      if hybridMode == 2
        controlData.linearTerm = -controlData.linearTerm;
      end
    end


    function plotDerivativeTrajectory(obj)
      figCounter = 60;
      fig = figure(figCounter);
      clf(fig);
      hold on;
      fnplt(obj.xtrajDot_, [3]);
      % fnplt(obj.xtrajDot_, [2]);
      title('x acceleration traj');
      hold off;
    end


  end

end