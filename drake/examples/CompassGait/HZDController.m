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
    dampingRatio = 1;
    S; % this stores the Lyapunov function corresponding to the PD controller

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
      defaultOptions.Kp = 20;
      defaultOptions.dampingRatio = 1.0;

      defaultOptions.applyUncertaintyControllerOnMakingContact = false;
      defaultOptions.applyUncertaintyControllerOnBreakingContact = false;
      defaultOptions.applyDeadZoneController = false;
      defaultOptions.applyWrongModeController = false;
      % defaultOptions.applyMakingContactWrongModeController = false;
      % defaultOptions.applyBreakingContactWrongModeController = false;

      hmu = struct();

      % the units of these two thresholds are not really comparable
      hmu.breakingContactTimeThreshold = 0.05;
      hmu.makingContactGuardThreshold = 0.1;
      hmu.makingContactToeHeightThreshold = 0.03;
      hmu.deadZoneControllerValue = 0; % this should be in the global coordinates, will be constant across both modes . . . 
      % defaultOptions.hybridModeUncertainty = hmu;

      if isfield(options, 'hybridModeUncertainty')
        options.hybridModeUncertainty = applyDefaults(options.hybridModeUncertainty, hmu);
      else
        options.hybridModeUncertainty = hmu;
      end

      options = applyDefaults(options, defaultOptions);
      obj.options = options;

      % some testing
      boolArray = [obj.options.applyDeadZoneController, obj.options.applyWrongModeController];

      if( sum(boolArray == true) > 1)
        error('only one of the two uncertainty controller options may be specified');
      end

    end


    function obj = setNominalTrajectory(obj, xtraj, utraj)
      obj.xtraj = xtraj;
      obj.utraj = utraj;
      obj = obj.setupController();
    end

    function obj = setupLyapunovFunction(obj)
      A = [0,1;0,0];
      B = [0;1];

      K = [obj.Kp, obj.Kd]; % the gain matrix, u = -K x

      temp = A - B*K;
      Q = eye(2);

      % this is a Lyapunov function for our PD controller
      obj.S = lyap(temp,Q);
    end

    % be sure to support things that aren't hybrid trajectories
    function obj = setupController(obj)

      obj.Kp = obj.options.Kp;
      obj.Kd = 2*sqrt(obj.Kp)*obj.options.dampingRatio;
      obj = obj.setupLyapunovFunction();

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
      [u, controlData] = getControlInput(obj,t,x);

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
    % this expects size(x)  = 5
    function [u, controlData] = getControlInput(obj,t,x)
      assert(length(x) == 5); % all the indices are supposing that x is of size 5

      % this is the standard control logic
      controlData = obj.getStandardControlInput(x);
      u = controlData.u;
      uStar = controlData.uStar;
      u_fb = controlData.u_fb;




      % The next sequence of if statements implements the custom logic for
      % the uncertainty based control stuff

      % there are three types of uncertainty that we could do
      [val, d] = obj.uncertaintyAboutHybridMode(t,x);


      if ((d.makingContact && obj.options.applyUncertaintyControllerOnMakingContact) || (d.breakingContact && obj.options.applyUncertaintyControllerOnBreakingContact))

        if (obj.options.applyDeadZoneController)
          u = obj.options.hybridModeUncertainty.deadZoneControllerValue;
        
          % these are just placeholders for when we are doing deadzone controller
          uStar = 0;
          u_fb = 0;
        end

        if (obj.options.applyWrongModeController)
          xOther = obj.cgUtils.transformStateToOtherMode(x);
          cData = obj.getStandardControlInput(x);

          % we need to flip the signs of all the control inputs, to get them into the
          % true local coordinates
          u = -cData.u;
          uStar = -cData.uStar;
          u_fb = -cData.u_fb;
        end

      end


      % if (val && obj.options.applyDeadZoneController)
      %   % this case means we should apply the dead zone style controller
      %   u = obj.options.hybridModeUncertainty.deadZoneControllerValue;
        
      %   % these are just placeholders for when we are doing deadzone controller
      %   uStar = 0;
      %   u_fb = 0; 

      % elseif ((d.makingContact && obj.options.applyMakingContactWrongModeController) || (d.breakingContact && obj.options.applyBreakingContactWrongModeController))
      %   % this means we are applying the controller for the wrong hybrid mode
      %   % and we are doing it either early or late depending on which flag is set

      %   % in either case we need to flip our state to the stateOther, because we should thinkg
      %   % of everything as being in some global coordinates and then us transforming it to
      %   % the current local coordinates depending on which is the stance leg
          
      %     xOther = obj.cgUtils.transformStateToOtherMode(x);
      %     cData = obj.getStandardControlInput(x);

      %     % we need to flip the signs of all the control inputs, to get them into the
      %     % true local coordinates
      %     u = -cData.u;
      %     uStar = -cData.uStar;
      %     u_fb = -cData.u_fb;
      % end
    

      % this records what really happens with the Lyapunov function for the u that we have chosen
      lyapunovData = obj.computeLyapunovData(x,u);

      % copy information from lyapunovData over into controlData struct
      for fn = fieldnames(lyapunovData)' % for some reason this prime is really important
        controlData.(fn{1}) = lyapunovData.(fn{1});
      end

    end


    function returnData = getStandardControlInput(obj, x)
      returnData = struct();
      lyapData = obj.computeLyapunovData(x,0);

      A_y = lyapData.A_y;
      B_y = lyapData.B_y;
      y = lyapData.y;
      ydot = lyapData.ydot;

      % just comes from the PD law
      yddot_des = -obj.Kp*y - obj.Kd*ydot;

      % yddot_des = A_y + B_y * u

      uStar = -A_y/B_y;
      u_fb = yddot_des/B_y;
      u = uStar + u_fb;


      returnData = applyDefaults(returnData, lyapData);

      returnData.u = u;
      returnData.uStar = uStar;
      returnData.u_fb = u_fb;
      returnData.yddot = yddot_des;
    end


    function trajStruct = makeTrajectoriesFromCellArray(obj,t,cellArray)
      singleDataStruct = cellArray{1};
      numTimes = length(t);

      fieldNames = fieldnames(singleDataStruct);

      dataArray = struct();
      for fn = fieldNames'
        dataArray.(fn{1}) = zeros(numTimes, length(singleDataStruct.(fn{1})));
      end

      for i=1:numTimes
        for fn = fieldNames'
          dataArray.(fn{1})(i,:) = cellArray{i}.(fn{1});
        end
      end

      trajStruct = struct();
      for fn = fieldNames'
        trajStruct.(fn{1}) = PPTrajectory(pchip(t, dataArray.(fn{1})));
      end

    end

    function controlTrajs = makeControlDataTrajectories(obj,tBreaks,dataCells)

      controlTrajs = struct();

      controlData = {};
      controlDataOther = {};


      for i=1:length(tBreaks)
        controlData{i} = dataCells{i}.controlData;
        controlDataOther{i} = dataCells{i}.controlDataOther;
      end

      controlTrajs.controlData = obj.makeTrajectoriesFromCellArray(tBreaks,controlData);
      controlTrajs.controlDataOther = obj.makeTrajectoriesFromCellArray(tBreaks,controlDataOther);



      % numTimes = length(obj.dataHandle.data.controlDataCells);

      % t = obj.dataHandle.data.times;
      % numTimes = length(t);
      % u = zeros(numTimes, 1);
      % uStar = zeros(numTimes,1);
      % u_fb = zeros(numTimes,1);
      % y = zeros(numTimes,1);
      % ydot = zeros(numTimes,1);
      % yddot = zeros(numTimes,1);
      % z = zeros(numTimes,1);
      % phaseVar = zeros(numTimes,1);
      % phaseVarClipped = zeros(numTimes,1);

      % S_val = zeros(numTimes,1);
      % S_dot = zeros(numTimes,1);


      % A_y = zeros(numTimes, 1);
      % B_y = zeros(numTimes,1);
      % wrongMode = struct();
      % wrongMode.u = zeros(numTimes,1);
      % wrongMode.yddot = zeros(numTimes,1);
      % wrongMode.S_dot = zeros(numTimes, 1);


      % for i=1:numTimes
      %   % t(i) = obj.dataHandle.data.controlDataCells{i}.t
      %   u(i) = controlDataCells{i}.u;
      %   uStar(i) = controlDataCells{i}.uStar;
      %   u_fb(i) = controlDataCells{i}.u_fb;
      %   y(i) = controlDataCells{i}.y;
      %   ydot(i) = controlDataCells{i}.ydot;
      %   yddot(i) = controlDataCells{i}.z;
      %   z(i) = controlDataCells{i}.z;
      %   phaseVar(i) = controlDataCells{i}.phaseVar;
      %   phaseVarClipped(i) = controlDataCells{i}.phaseVarClipped;

      %   S_val(i) = controlDataCells{i}.S_val;
      %   S_dot(i) = controlDataCells{i}.S_dot_val;

      %   wrongMode.u(i) = controlDataCells{i}.otherHybridMode.u;
      %   wrongMode.yddot(i) = controlDataCells{i}.wrongHybridMode.yddot;
      %   wrongMode.S_dot(i) = controlDataCells{i}.wrongHybridMode.S_dot_val;
      % end

      % controlTrajs = struct();
      % % controlTrajs.traj.t = PPTrajectory(pchip(t, t));
      % controlTrajs.traj.u = PPTrajectory(pchip(t,u));
      % controlTrajs.traj.uStar = PPTrajectory(pchip(t, uStar));
      % controlTrajs.traj.u_fb = PPTrajectory(pchip(t, u_fb));
      % controlTrajs.traj.y = PPTrajectory(pchip(t, y));
      % controlTrajs.traj.ydot = PPTrajectory(pchip(t, ydot));
      % controlTrajs.traj.z = PPTrajectory(pchip(t, z));
      % controlTrajs.traj.yddot = controlTrajs.traj.z;
      % controlTrajs.traj.phaseVar = PPTrajectory(pchip(t, phaseVar));
      % controlTrajs.traj.phaseVarClipped = PPTrajectory(pchip(t, phaseVarClipped));

      % controlTrajs.traj.S = PPTrajectory(pchip(t, S_val));
      % controlTrajs.traj.S_dot = PPTrajectory(pchip(t, S_dot));

      % controlTrajs.traj.wrongMode.u = PPTrajectory(pchip(t, wrongMode.u));
      % controlTrajs.traj.wrongMode.yddot = PPTrajectory(pchip(t, wrongMode.yddot));
      % controlTrajs.traj.wrongMode.S_dot = PPTrajectory(pchip(t, wrongMode.S_dot));

      % controlTrajs.grid.t = t;
      % controlTrajs.grid.u = u;
      % controlTrajs.grid.uStar = uStar;
      % controlTrajs.grid.u_fb = u_fb;
      % controlTrajs.grid.y = y;
      % controlTrajs.grid.ydot = ydot;
      % controlTrajs.grid.yddot = z;
      % controlTrajs.grid.z = z;
      % controlTrajs.grid.phaseVar = phaseVar;
      % controlTrajs.grid.phaseVarClipped = phaseVarClipped;
    end

    function data = recomputeControlData(obj, t, ytrajVal)

      assert(length(ytrajVal) == 6);

      x = ytrajVal(1:5);
      u = ytrajVal(end);

      controlOptions.u = u;
      controlData = obj.computeLyapunovData(x,u);

      xOther = obj.cgUtils.transformStateToOtherMode(x);
      uOther = -u;

      controlOptions.u = uOther;
      controlDataOther = obj.computeLyapunovData(xOther,uOther);

      
      data.controlData = controlData;
      data.controlDataOther = controlDataOther;
    end


    function returnData = reconstructControlDataFromTrajectory(obj, ytraj)
      tBreaks = ytraj.getBreaks();
      numTimes = length(tBreaks);
      controlDataCells = {};

      % we assume that u is the final field of ytraj.

      for i=1:numTimes
        t = tBreaks(i);
        controlDataCells{end+1} = obj.recomputeControlData(t, ytraj.eval(t));
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
      phaseVarUnclipped = phaseVar;
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
      returnData.phaseVarUnclipped = phaseVarUnclipped;
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
      toeHeight = obj.compassGaitPlant.computeToeHeight(x);

      theta_st = x(obj.stanceIdx);
      theta_sw = x(obj.swingIdx);

      % this is the interleg angle
      % DEPRECATED
      % sig = theta_sw - theta_st;


      % this should only happen if stance leg angle is sufficiently negative
      if ((theta_st < -0.02) && (toeHeight < obj.options.hybridModeUncertainty.makingContactToeHeightThreshold))

        val = true;
        returnData.makingContact = true;
        returnData.guard1 = guard1;
        returnData.toeHeight = toeHeight;
        return;
      end

    end

    function data = computeLyapunovData(obj,x,u,hzdOutputData)
      if (nargin < 4)
        hzdOutputData = obj.computeHybridZeroDynamics(x);
      end

      y = hzdOutputData.y;
      ydot = hzdOutputData.ydot;
      A_y = hzdOutputData.A_y;
      B_y = hzdOutputData.B_y;
      yddot = A_y + B_y*u;

      % value of Lyapunov function
      V = [y, ydot] * obj.S * [y;ydot];
      V_dot = 2*[y, ydot] * obj.S * [ydot;yddot];

      data = struct();
      data.u = u;
      data.uStar = -A_y/B_y;
      data.y = y;
      data.ydot = ydot;
      data.A_y = A_y;
      data.B_y = B_y;
      data.yddot = yddot;
      data.V = V;
      data.V_dot = V_dot;
    end


    % DEPRECATED
    % function data = computeLyapunovFunctionInformation(obj, t, x,u,controlData)
    %   data = controlData; % just add some fields to it
    %   assert(length(x) == 5);

    %   % [y, ydot] * S * [y; ydot]
    %   data.S_val = [controlData.y; controlData.ydot]' * obj.S * [controlData.y; controlData.ydot];

    %   % [y, ydot] * S * [ydot; yddot]
    %   data.S_dot_val = 2*[controlData.y; controlData.ydot]' * obj.S * [controlData.ydot; controlData.yddot];


    %   % compute what we would have done if we (mistakenly) though we were in the other mode
    %   [u_other, controlDataOther] = obj.testGetControlInput(t, x);

    %   % the next to lines I put in for completeness. They don't really mean much at the moment
    %   % what really matters is the control input u that we would have done if we thought that
    %   % we were in that mode
    %   controlDataOther.S_val = [controlDataOther.y; controlDataOther.ydot]' * obj.S * [controlDataOther.y; controlDataOther.ydot];
    %   controlDataOther.S_dot_val = 2 * [controlData.y; controlData.ydot]' * obj.S * [controlDataOther.ydot; controlDataOther.yddot];

    %   data.otherHybridMode = controlDataOther;


    %   % this determines what happens if we are in the wrong hybrid mode, i.e. the true state is
    %   % x but we control as if we were in the other mode
    %   hzDynamics = obj.computeHybridZeroDynamics(x); % this is the hzdDynamics in the current hybrid mode

    %   data.A_y = hzDynamics.A_y;
    %   data.B_y = hzDynamics.B_y;
    %   data.wrongHybridMode.yddot = data.A_y + data.B_y*u_other;
    %   data.wrongHybridMode.S_dot_val = 2 * [controlData.y; controlData.ydot]' * obj.S * [controlData.ydot; data.wrongHybridMode.yddot];
    % end





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