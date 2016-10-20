classdef LQRController < SimpleController


  properties
    cgLQR;
    xtraj; % nominal state trajectory 
    utraj; % nominal control trajectory
    xPhaseTraj;
    uPhaseTraj;
    tPhaseTraj;
    S_traj; % stores LQR value function, in trajectory form
    compassGaitStancePlant;
    options;
    phaseIdx = 3;
  end

  methods
    function obj = LQRController(compassGaitPlant, options)

      if nargin < 2
        options = struct();
      end

      obj = obj@SimpleController(compassGaitPlant);
      obj.compassGaitStancePlant = compassGaitPlant.modes{1};

      obj.initializeDataHandle();
      obj = obj.initializeOptions(options);

    end

    function obj = initializeDataHandle(obj)
      % stores time at which we last reset the plan time
      obj.dataHandle.data.lastPlanResetTimeGlobal = -1;

      % should be set according to phase variable
      obj.dataHandle.data.lastPlanResetTimeOffsetGlobal = 0;  

      % stores mode for which last reset happened
      obj.dataHandle.data.lastPlanResetModeGlobal = -1;

      obj.dataHandle.data.lastPlanResetTime = {-1,-1};
      obj.dataHandle.data.lastPlanResetTimeOffset = {-1,-1};
    end

    function obj = initializeOptions(obj, options)
      defaultOptions = struct();
      defaultOptions.Q = eye(4); % for LQR state cost
      defaultOptions.R = 0.1; % for LQR action cost


      options = applyDefaults(options, defaultOptions);
      obj.options = options;
    end

    % this xtraj should be 4 x 1, shouldn't include mode, only a single step
    function obj = setNominalTrajectory(obj, xtraj, utraj)
      obj.xtraj = xtraj;
      obj.utraj = utraj;
      obj = obj.computePhaseTrajectory();
      obj = obj.setupController();
    end


    function obj = setupController(obj)
      obj = obj.computeLQRValueFunction();
    end
    

    function obj = computePhaseTrajectory(obj)

      x_test = obj.xtraj.eval(0);
      assert(length(x_test)==4);

      stanceLegIdx = 2;

      tBreaks = obj.xtraj.getBreaks();
      xGrid = obj.xtraj.eval(tBreaks);
      phaseGrid = xGrid(stanceLegIdx, :); % the stance leg angles throughout the trajectory

      uGrid = obj.utraj.eval(tBreaks);

      obj.xPhaseTraj = PPTrajectory(pchip(phaseGrid, xGrid));
      obj.tPhaseTraj = PPTrajectory(pchip(phaseGrid, tBreaks));
      obj.uPhaseTraj = PPTrajectory(pchip(phaseGrid, uGrid));
    end

    function obj = computeLQRValueFunction(obj)
      obj.cgLQR = CompassGaitLQR(obj.compassGaitPlant, obj.xtraj, obj.utraj);
      [tGrid, S_grid] = obj.cgLQR.computeLQRValueFunction(obj.options.Q, obj.options.R);


      % now make a trajectory from it
      S_temp = permute(S_grid, [2 3 1]); 
      obj.S_traj = PPTrajectory(pchip(tGrid, S_temp));
    end


    function t_plan = computePlanTime(obj,t,x)
      

      % this assumes that x is of size 5
      assert(length(x)==5);
      phaseVar = x(obj.phaseIdx);
      phaseSpan = obj.tPhaseTraj.tspan;

      % clip the phase to limits
      phaseVar = min(phaseVar, phaseSpan(2));
      phaseVar = max(phaseVar, phaseSpan(1));
      modeVar = x(1);

      % this means it is the first time we are hitting this
      % so we handle this special case differently.
      if(obj.dataHandle.data.lastPlanResetModeGlobal < 0)
        modeVar = x(1);
        obj.dataHandle.data.lastPlanResetModeGlobal = x(1);
        obj.dataHandle.data.lastPlanResetTimeGlobal = t;

        

        t_offset = obj.tPhaseTraj.eval(phaseVar);

        obj.dataHandle.data.lastPlanResetTimeOffsetGlobal = t_offset;

        obj.dataHandle.data.lastPlanResetTime{modeVar} = t;
        obj.dataHandle.data.lastPlanResetTimeOffset{modeVar} = t_offset;
        t_plan = t_offset + (t - obj.dataHandle.data.lastPlanResetTimeGlobal);
        obj.dataHandle.data.currentPlanTime = t_plan;
        return;
      end


      % You haven't switched modes since last reset
      if (obj.dataHandle.data.lastPlanResetModeGlobal == x(1))
        % this means we are in the same mode as last reset the time
        t_prev = obj.dataHandle.data.lastPlanResetTimeGlobal;

        % if t > t_prev, then there is no need to do anything

        % if t < t_prev, then the optimizer has jumped back in time to before the reset so you should redo that reset
        if (t < t_prev)
          obj.dataHandle.data.lastPlanResetModeGlobal = modeVar;
          obj.dataHandle.data.lastPlanResetTimeGlobal = t;

          phaseVar = x(obj.phaseIdx);
          phaseSpan = obj.tPhaseTraj.tspan;

          % clip the phase to limits
          phaseVar = min(phaseVar, phaseSpan(2));
          phaseVar = max(phaseVar, phaseSpan(1));

          t_offset = obj.tPhaseTraj.eval(phaseVar);
          obj.dataHandle.data.lastPlanResetTimeOffsetGlobal = t_offset;

          obj.dataHandle.data.lastPlanResetTime{modeVar} = t;
          obj.dataHandle.data.lastPlanResetTimeOffset{modeVar} = t_offset;
        end
      else
        % in this case you are in a different mode.

        % if we have just jumped (forwards) into this new mode, then 
        % we should reset everything
        t_prev = obj.dataHandle.data.lastPlanResetTimeGlobal;

        if (t_prev <= t)
          obj.dataHandle.data.lastPlanResetModeGlobal = modeVar;
          obj.dataHandle.data.lastPlanResetTimeGlobal = t;

          phaseVar = x(obj.phaseIdx);
          phaseSpan = obj.tPhaseTraj.tspan;

          % clip the phase to limits
          phaseVar = min(phaseVar, phaseSpan(2));
          phaseVar = max(phaseVar, phaseSpan(1));

          t_offset = obj.tPhaseTraj.eval(phaseVar);
          obj.dataHandle.data.lastPlanResetTimeOffsetGlobal = t_offset;

          obj.dataHandle.data.lastPlanResetTime{modeVar} = t;
          obj.dataHandle.data.lastPlanResetTimeOffset{modeVar} = t_offset;
        else
          % this means we have jumped back in time, but to a different mode.
          % the only way this is possible is if we jumped back across a hybrid mode switch

          % in this case we should use what we were using before in that mode
          % which is stored in the dataHandle data for that mode
          obj.dataHandle.data.lastPlanResetTimeGlobal = obj.dataHandle.data.lastPlanResetTime{modeVar};

          obj.dataHandle.data.lastPlanResetTimeOffsetGlobal = obj.dataHandle.data.lastPlanResetTimeOffset{modeVar};

          obj.dataHandle.data.lastPlanResetModeGlobal = modeVar;
        end
      end

      t_plan = obj.dataHandle.data.lastPlanResetTimeOffsetGlobal + (t-obj.dataHandle.data.lastPlanResetTimeGlobal);

      obj.dataHandle.data.currentPlanTime = t_plan;

    end

    function obj = step(obj,t,x,u)
      obj.dataHandle.data.tickCounter = obj.dataHandle.data.tickCounter + 1;
      obj.dataHandle.data.times = [obj.dataHandle.data.times;t];

      t_plan = obj.computePlanTime(t,x);
      u = obj.computeControlInput(t,x,t_plan);

      % bookkeeping: store some control data
      obj.dataHandle.data.currentControlInput = u;

      % for debugging only
      % use this to run with passive compass gait to check piping
      obj.dataHandle.data.currentControlInput = 0;

      obj.dataHandle.data.u(end+1) = u;
      modeVal = x(1);
      obj.dataHandle.data.lastTimeInEachMode(modeVal) = t;
    end


    function u = computeControlInput(obj,t,x_full,t_plan)


      % clip t_plan to limits
      tspan = obj.S_traj.tspan;
      t_plan = min(t_plan, tspan(2));
      t_plan = max(t_plan, tspan(1));

      % the LQR value function
      S = obj.S_traj.eval(t_plan);

      % the part of the state corresponding to the stance plant
      x = x_full(2:end); 


      % now linearize the plant around the current state from the plan to
      % get the control input

      x_des =  obj.xtraj.eval(t_plan);
      [A,B] = obj.compassGaitStancePlant.linearize(t_plan, x_des, 0);

      % control input consists of feedback and feedforward
      u_fb = -obj.options.R^(-1)*B'*S*(x - x_des);
      u_ff = obj.utraj.eval(t_plan);

      u = u_fb + u_ff;

    end

    function u = getCurrentControlInput(obj)
      u = obj.dataHandle.data.currentControlInput;
    end

  end 



end