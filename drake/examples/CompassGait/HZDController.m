classdef HZDController < SimpleController
  
  properties
    compassGaitStancePlant;
    xPhaseTraj;
    uPhaseTraj;
    xdotTrajPhase;
    theta1_idx = 2; % note this assumes that mode is in x, i.e. x in R^5
    theta1dot_idx = 4;
    stanceIdx = 3;
    swingIdx = 2;
    theta2dot_idx = 5;
    Kp = 10;
    Kd = 6;

    hdTraj;
    hdTraj_deriv;
    hdTraj_dderiv;

    xtraj; % the nominal trajectory that is passed in
    utraj;

    cgUtils;
    options;
  end
  
  methods
    function obj = HZDController(compassGaitPlant, options)
      if nargin < 2
        options = struct()
      end


      obj = obj@SimpleController(compassGaitPlant);
      obj.compassGaitStancePlant = compassGaitPlant.modes{1};
      obj.initializeDataHandle();
      obj.cgUtils = CompassGaitUtils();
      obj = obj.initializeOptions(options);
    end

    function obj = initializeDataHandle(obj)
      obj.dataHandle.data.times = [];
      obj.dataHandle.data.u_ff = [];
      obj.dataHandle.data.u_fb = [];
      obj.dataHandle.data.z = [];
      obj.dataHandle.data.controlDataCells = {};
      obj.dataHandle.data.lastTimeInEachMode = -ones(2,1);
    end

    function obj = initializeOptions(obj, options)
      defaultOptions = struct();
      hmu = struct();

      % the units of these two thresholds are not really comparable
      hmu.breakingContactTimeThreshold = 0.05;
      hmu.makingContactGuardThreshold = 0.1;
      hmu.applyDeadZoneController = false;
      hmu.deadZoneControllerValue = 0; % this should be in the global coordinates, will be constant across both modes . . . 
      defaultOptions.hybridModeUncertainty = hmu;

      options = applyDefaults(options, defaultOptions);
      obj.options = options;
    end


    function obj = setNominalTrajectory(obj, xtraj, utraj)
      obj.xtraj = xtraj;
      obj.utraj = utraj;
      obj = obj.setupController();
    end

    % be sure to support things that aren't hybrid trajectories
    function obj = setupController(obj)

      % this means it is a regular trajectory and we don't need to do all the mess from below
      if (~isa(obj.xtraj,'HybridTrajectory'))
        [h,hDeriv,hDeriv2, xPhaseTrajTemp, uPhaseTrajTemp] = obj.compute_H_Trajectory(obj.xtraj, obj.utraj);
        obj.hdTraj = h;
        obj.hdTraj_deriv = hDeriv;
        obj.hdTraj_dderiv = hDeriv2;
        obj.uPhaseTraj = uPhaseTrajTemp;
        obj.xPhaseTraj = xPhaseTrajTemp;
        return;
      end


      numTrajectories = length(obj.xtraj.traj);
      h_trajs = {};
      h_deriv_trajs = {};
      h_deriv2_trajs = {};
      uPhaseTraj = {};
      xPhaseTraj = {};




      % only need to do this stuff if it's a hybrid trajectory thing
      for i=1:numTrajectories
        traj = obj.xtraj.traj{i}.trajs{2}; % remove the 
        uTraj = obj.utraj.traj{i};
        [h,hDeriv,hDeriv2, xPhaseTrajTemp, uPhaseTrajTemp] = obj.compute_H_Trajectory(traj, uTraj);
        h_trajs{i} = h;
        h_deriv_trajs{i} = hDeriv;
        h_deriv2_trajs{i} = hDeriv2;
        uPhaseTraj{i} = uPhaseTrajTemp;
        xPhaseTraj{i} = xPhaseTrajTemp;
      end
      
      % ensure that the trajectories are in the same frames
      % otherwise HybridTrajectory will complain
      for i=1:numTrajectories
        h_trajs{i} = h_trajs{i}.setOutputFrame(h_trajs{1}.getOutputFrame);
        h_deriv_trajs{i} = h_deriv_trajs{i}.setOutputFrame(h_deriv_trajs{1}.getOutputFrame);
        h_deriv2_trajs{i} = h_deriv2_trajs{i}.setOutputFrame(h_deriv2_trajs{1}.getOutputFrame);
        uPhaseTraj{i} = uPhaseTraj{i}.setOutputFrame(uPhaseTraj{1}.getOutputFrame);
        xPhaseTraj{i} = xPhaseTraj{i}.setOutputFrame(xPhaseTraj{1}.getOutputFrame);
      end

      obj.hdTraj = HybridTrajectory(h_trajs);
      obj.hdTraj_deriv = HybridTrajectory(h_deriv_trajs);
      obj.hdTraj_dderiv = HybridTrajectory(h_deriv2_trajs);
      obj.uPhaseTraj = HybridTrajectory(uPhaseTraj);
      obj.xPhaseTraj = HybridTrajectory(xPhaseTraj);
    end

    % x is a trajectory here, shouldn't include the mode
    % just has (theta_sw, theta_st, theta_sw_dot, theta_st_dot)
    function [h, hDeriv, hDeriv2, xPhaseTraj, uPhaseTraj] = compute_H_Trajectory(obj, x, u)
      tBreaks = x.getBreaks();
      xGrid = x.eval(tBreaks);
      xDeriv = x.fnder(1);
      xDeriv2 = x.fnder(2);

      xTrueGrid = x.eval(tBreaks);
      xDerivTrueGrid = xDeriv.eval(tBreaks);

      hDerivTrueGrid = xTrueGrid(3,:)./xTrueGrid(4,:);

      hDeriv2TrueGrid = xDerivTrueGrid(3,:)./xTrueGrid(4,:).^2 - ...
        hDerivTrueGrid.*(xDerivTrueGrid(4,:))./xTrueGrid(4,:).^2;

      uTrueGrid = u.eval(tBreaks);

      phaseGrid = xGrid(2,:);
      h = PPTrajectory(pchip(phaseGrid, xGrid(1,:)));
      hDeriv = PPTrajectory(pchip(phaseGrid, hDerivTrueGrid));
      hDeriv2 = PPTrajectory(pchip(phaseGrid, hDeriv2TrueGrid));
      uPhaseTraj = PPTrajectory(pchip(phaseGrid, uTrueGrid));
      xPhaseTraj = PPTrajectory(pchip(phaseGrid, xTrueGrid));
    end

    % indexed by theta_1 (swing leg angle), rather than time
    function obj = setTrajectory(obj, xtrajPhase, utrajPhase, xdotTrajPhase)
        obj.xtrajPhase = xtrajPhase;
        obj.utrajPhase = utrajPhase;
        obj.xdotTrajPhase = xdotTrajPhase;

        obj.hdTraj = xtrajPhase(obj.theta1_idx);
        obj.hdTraj_deriv = obj.hdTraj.fnder(1);
        obj.hdTraj_dderiv = obj.hdTraj.fnder(2);
    end
    
    function obj = step(obj,t,x,u)
      obj.dataHandle.data.tickCounter = obj.dataHandle.data.tickCounter + 1;
      obj.dataHandle.data.times = [obj.dataHandle.data.times;t];
      [u, controlData] = testGetControlInput(obj,t,x);

      % bookkeeping: store some control data
      obj.dataHandle.data.currentControlInput = u;
      obj.dataHandle.data.u(end+1) = u;
      obj.dataHandle.data.controlDataCells{end+1} = controlData;

      modeVal = x(1);
      obj.dataHandle.data.lastTimeInEachMode(modeVal) = t;
    end
    
    function u = getCurrentControlInput(obj)
      u = obj.dataHandle.data.currentControlInput;
    end


    % this could be simplified a lot by using the new computeHybridZeroDyanmics method
    function [u, controlData] = testGetControlInput(obj,t,x, verbose)
      if nargin < 4
        verbose = false;
      end

      controlData = struct();
      % Figure out what plan is telling us to do
      phaseVar = x(obj.stanceIdx);
      controlData.phaseVar = phaseVar;

      % clip the phaseVar to limits
      % could this be messing us up? maybe need to do a linear interpolation
      % off of the ends
      tspan = obj.hdTraj.tspan;
      phaseVar = max(tspan(1), min(tspan(2), phaseVar));

      thetaSwing_des = obj.hdTraj.eval(phaseVar);

      theta1 = x(obj.theta1_idx);
      theta1dot = x(obj.theta1dot_idx);
      theta2dot = x(obj.theta2dot_idx);

      h_deriv = [1, -obj.hdTraj_deriv.eval(phaseVar)];
      h_d_dderiv = obj.hdTraj_dderiv.eval(phaseVar);

      y = theta1 - thetaSwing_des;
      ydot = h_deriv*[theta1dot; theta2dot];


      % desired acceleration of y
      % this is just the PD law
      z = -obj.Kp*y - obj.Kd*ydot; 


      q = x(2:3);
      v = x(4:5);
      [H,C,B] = obj.compassGaitStancePlant.manipulatorDynamics(q,v);

      temp = (h_deriv*inv(H)*B)^(-1);
      uStar = temp*(h_d_dderiv*theta2dot^2 + h_deriv*inv(H)*C);
      u_fb = temp*z;

      u = uStar + u_fb;

      % see if we need to do the uncertainty based controller thing
      [val, d] = obj.uncertaintyAboutHybridMode(t,x);
      if (val && obj.options.hybridModeUncertainty.applyDeadZoneController)
        u = obj.options.hybridModeUncertainty.deadZoneControllerValue;
        uStar = 0;
        u_fb = 0;
      end


      controlData.u = u;
      controlData.uStar = uStar;
      controlData.u_fb = u_fb;
      controlData.y = y;
      controlData.ydot = ydot;
      controlData.z = z;
      controlData.phaseVarClipped = phaseVar;



    end


    % function controlTrajs = makeControlDataTrajectories(obj,t,controlDataCells)
    %   % numTimes = length(obj.dataHandle.data.controlDataCells);

    %   % t = obj.dataHandle.data.times;
    %   numTimes = length(t);
    %   uStar = zeros(numTimes);
    %   u_fb = zeros(numTimes);
    %   y = zeros(numTimes);
    %   ydot = zeros(numTimes);
    %   z = zeros(numTimes);
    %   phaseVar = zeros(numTimes);
    %   phaseVarClipped = zeros(numTimes);

    %   for i=1:numTimes
    %     % t(i) = obj.dataHandle.data.controlDataCells{i}.t
    %     uStar(i) = obj.dataHandle.data.controlDataCells{i}.uStar;
    %     u_fb(i) = obj.dataHandle.data.controlDataCells{i}.u_fb;
    %     y(i) = obj.dataHandle.data.controlDataCells{i}.y;
    %     ydot(i) = obj.dataHandle.data.controlDataCells{i}.ydot;
    %     z(i) = obj.dataHandle.data.controlDataCells{i}.z;
    %     phaseVar(i) = obj.dataHandle.data.controlDataCells{i}.phaseVar;
    %     phaseVarClipped(i) = obj.dataHandle.data.controlDataCells{i}.phaseVarClipped;
    %   end

    %   controlTrajs = struct();
    %   % controlTrajs.traj.t = PPTrajectory(pchip(t, t));
    %   controlTrajs.traj.uStar = PPTrajectory(pchip(t, uStar));
    %   controlTrajs.traj.u_fb = PPTrajectory(pchip(t, u_fb));
    %   controlTrajs.traj.y = PPTrajectory(pchip(t, y));
    %   controlTrajs.traj.ydot = PPTrajectory(pchip(t, ydot));
    %   controlTrajs.traj.z = PPTrajectory(pchip(t, z));
    %   controlTrajs.traj.phaseVar = PPTrajectory(pchip(t, phaseVar));
    %   controlTrajs.traj.phaseVarClipped = PPTrajectory(pchip(t, phaseVarClipped));

    %   controlTrajs.grid.t = t;
    %   controlTrajs.grid.uStar = uStar;
    %   controlTrajs.grid.u_fb = u_fb;
    %   controlTrajs.grid.y = y;
    %   controlTrajs.grid.ydot = ydot;
    %   controlTrajs.grid.z = z;
    %   controlTrajs.grid.phaseVar = phaseVar;
    %   controlTrajs.grid.phaseVarClipped = phaseVarClipped;
    % end

    function controlTrajs = makeControlDataTrajectories(obj,t,controlDataCells)
      % numTimes = length(obj.dataHandle.data.controlDataCells);

      % t = obj.dataHandle.data.times;
      numTimes = length(t);
      u = zeros(numTimes, 1);
      uStar = zeros(numTimes,1);
      u_fb = zeros(numTimes,1);
      y = zeros(numTimes,1);
      ydot = zeros(numTimes,1);
      z = zeros(numTimes,1);
      phaseVar = zeros(numTimes,1);
      phaseVarClipped = zeros(numTimes,1);

      for i=1:numTimes
        % t(i) = obj.dataHandle.data.controlDataCells{i}.t
        u(i) = controlDataCells{i}.u;
        uStar(i) = controlDataCells{i}.uStar;
        u_fb(i) = controlDataCells{i}.u_fb;
        y(i) = controlDataCells{i}.y;
        ydot(i) = controlDataCells{i}.ydot;
        z(i) = controlDataCells{i}.z;
        phaseVar(i) = controlDataCells{i}.phaseVar;
        phaseVarClipped(i) = controlDataCells{i}.phaseVarClipped;
      end

      controlTrajs = struct();
      % controlTrajs.traj.t = PPTrajectory(pchip(t, t));
      controlTrajs.traj.u = PPTrajectory(pchip(t,u));
      controlTrajs.traj.uStar = PPTrajectory(pchip(t, uStar));
      controlTrajs.traj.u_fb = PPTrajectory(pchip(t, u_fb));
      controlTrajs.traj.y = PPTrajectory(pchip(t, y));
      controlTrajs.traj.ydot = PPTrajectory(pchip(t, ydot));
      controlTrajs.traj.z = PPTrajectory(pchip(t, z));
      controlTrajs.traj.phaseVar = PPTrajectory(pchip(t, phaseVar));
      controlTrajs.traj.phaseVarClipped = PPTrajectory(pchip(t, phaseVarClipped));

      controlTrajs.grid.t = t;
      controlTrajs.grid.u = u;
      controlTrajs.grid.uStar = uStar;
      controlTrajs.grid.u_fb = u_fb;
      controlTrajs.grid.y = y;
      controlTrajs.grid.ydot = ydot;
      controlTrajs.grid.z = z;
      controlTrajs.grid.phaseVar = phaseVar;
      controlTrajs.grid.phaseVarClipped = phaseVarClipped;
    end


    function returnData = reconstructControlDataFromTrajectory(obj, xtraj)
      tBreaks = xtraj.getBreaks();
      numTimes = length(tBreaks);
      controlDataCells = {};

      for i=1:numTimes
        t = tBreaks(i);
        [u,controlData] = obj.testGetControlInput(t, xtraj.eval(t));
        controlDataCells{end+1} = controlData;
      end

      returnData = obj.makeControlDataTrajectories(tBreaks, controlDataCells);
    end


    % this will all be in global coordinates
    % does it for true mode as well as "pretending" to be in either right/left mode
    % this x should be the local state
    function returnData = manipulatorDynamicsFromState(obj, x)
      returnData = struct();
      assert(length(x) == 5);
      [x_global,d] = obj.cgUtils.transformStateToGlobal(x);


      % compute global with true mode first
      q = x(2:3);
      v = x(4:5);
      [H,C,B] = obj.compassGaitStancePlant.manipulatorDynamics(q,v);

      [H_global, C_global, B_global] = obj.cgUtils.transformDynamics(x,H,C,B);


      returnData.global.H = H_global;
      returnData.global.C = C_global;
      returnData.global.B = B_global;


      % now compute global with mode set to either left/right
      [H_left, C_left, B_left] = obj.compassGaitStancePlant.manipulatorDynamics(d.left.q, d.left.v);

      [H_left, C_left, B_left] = obj.cgUtils.transformDynamics(d.left.x, H_left, C_left, B_left);

      returnData.left.H = H_left;
      returnData.left.C = C_left;
      returnData.left.B = B_left;

      % do it for right as well
      [H_right, C_right, B_right] = obj.compassGaitStancePlant.manipulatorDynamics(d.right.q, d.right.v);

      [H_right, C_right, B_right] = obj.cgUtils.transformDynamics(d.right.x, H_right, C_right, B_right);


      returnData.right.H = H_right;
      returnData.right.C = C_right;
      returnData.right.B = B_right;

    end


    % note u must be in global here
    function returnData = dynamicsInBothModes(obj, x, u_global)
      d = obj.manipulatorDynamicsFromState(x);

      returnData = struct();

      % first do left
      A_temp = inv(d.left.H)*d.left.C;
      B_temp = inv(d.left.H)*d.left.B;

      returnData.left.qddot_global = A_temp + B_temp*u_global;


      % now do right
      A_temp = inv(d.right.H)*d.right.C;
      B_temp = inv(d.right.H)*d.right.B;

      returnData.right.qddot_global = A_temp + B_temp*u_global;

    end

    % Computes the dynamics of y. Will be of the form yddot = A + B u_global instantaneously
    % x is in local coordinates here
    function returnData = zeroDynamicsBothModes(obj, x, options)

      defaultOptions = struct();
      defaultOptions.u_in_global_coords = true;

      if (nargin < 3)
        options = struct();
      end

      options = applyDefaults(options, defaultOptions);

      [x_global, d] = obj.cgUtils.transformStateToGlobal(x);
      returnData = struct();
      returnData.right = obj.computeHybridZeroDynamics(d.right.x);
      returnData.left = obj.computeHybridZeroDynamics(d.left.x); 


      if (options.u_in_global_coords)
        returnData.left.B_y = -returnData.left.B_y;
      end


    end


    % this assumes x is in local coordinates
    % note this also leaves u in local coordinates as well
    % to get it into global coordinates you need to reverse the sign of B if 
    % we are in mode 2.
    function returnData = computeHybridZeroDynamics(obj, x)
      assert(length(x) == 5);

      phaseVar = x(obj.stanceIdx);
      % clip the phaseVar to limits
      % could this be messing us up? maybe need to do a linear interpolation
      % off of the ends
      tspan = obj.hdTraj.tspan;
      phaseVar = max(tspan(1), min(tspan(2), phaseVar));

      thetaSwing_des = obj.hdTraj.eval(phaseVar);

      theta1 = x(obj.theta1_idx);
      theta1dot = x(obj.theta1dot_idx);
      theta2dot = x(obj.theta2dot_idx);

      h_deriv = [1, -obj.hdTraj_deriv.eval(phaseVar)];
      h_d_dderiv = obj.hdTraj_dderiv.eval(phaseVar);

      y = theta1 - thetaSwing_des;
      ydot = h_deriv*[theta1dot; theta2dot];


      % get the manipulator dynamics
      q = x(2:3);
      v = x(4:5);
      [H,C,B] = obj.compassGaitStancePlant.manipulatorDynamics(q,v);


      A_y = -h_d_dderiv*theta2dot^2  - h_deriv*inv(H)*C;
      B_y = h_deriv*inv(H)*B;

      returnData = struct();
      returnData.A_y = A_y;
      returnData.B_y = B_y;
      returnData.y = y;
      returnData.ydot = ydot;
      returnData.phaseVar = phaseVar;
    end


    % helper function to compute the "fake" uncertainty that we have about 
    function [val, returnData] = uncertaintyAboutHybridMode(obj,t,x)
      val = false;
      returnData = struct();
      returnData.breakingContact = false;
      returnData.makingContact = false;

      % should specify the type of uncertainty, whether making contact or breaking contact
      xs = x(2:end); % just the cts part of the state

      currentMode = x(1);
      otherMode = 3 - currentMode;

      % first check if we are breaking contact
      tPrev = obj.dataHandle.data.lastTimeInEachMode(otherMode);
      if ( (t > tPrev) && (t - tPrev) < obj.options.hybridModeUncertainty.breakingContactTimeThreshold)
        % this means we are in the breaking contact uncertainty type
        val = true;
        returnData.breakingContact = true;
        returnData.timeVal = t - tPrev;
        return;
      end


      % now see if we are in the making contact mode
      guard1 = obj.compassGaitPlant.footCollisionGuard1(t,xs,0);
      guard2 = obj.compassGaitPlant.footCollisionGuard2(t,xs,0);

      theta_st = x(obj.stanceIdx);
      theta_sw = x(obj.swingIdx);

      % this is the interleg angle
      sig = theta_sw - theta_st;

      % this should only happen if interleg angle is sufficiently large
      if ((sig > 0.14) && (guard1 < obj.options.hybridModeUncertainty.makingContactGuardThreshold))

        val = true;
        returnData.makingContact = true;
        returnData.guard1 = guard1;
        return;
      end








    end





    % % theta1ddot is the desired acceleration of theta1
    % function u_fl = computeFeedbackLinearization(obj,t,x_full,theta1ddot)
    %   x = x_full(2:end);
    %   q = x(1:2);
    %   v = x(3:4);
    %   [H,C,B] = obj.compassGaitStancePlant.manipulatorDynamics(q,v);
    %   H_inv = inv(H);
    %   a = -H_inv*C;
    %   a = a(1);

    %   b = H_inv*B;
    %   b = b(1);

    %   % then we have 
    %   % z = theta_1_ddot = a + bu
      
    %   u_fl = (theta1ddot - a)/b;
    % end




  end
  
end