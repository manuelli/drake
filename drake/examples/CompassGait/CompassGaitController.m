classdef CompassGaitController < DrakeSystem
  
  properties
    compassGaitPlant;
    currentControlInput = 0.0;
    ytraj_des;
    lqrControllers = {};
    numStates;
    numDiscStates;
    numContStates;
    numControlInputs
    trajTimes;
    Q; % cost on state
    R; % cost on controlInput
    Qf; % final cost for LQR
    tickCounter = 0;
    controlBasedOnSensedMode = false;
  end
  
  methods 
    function obj = CompassGaitController(compassGaitPlant)
      numInputs = compassGaitPlant.getNumOutputs;
      numOutputs = compassGaitPlant.getNumInputs;
      directFeedthroughFlag = true;
      timeInvariantFlag = false;
      obj = obj@DrakeSystem(0,0,numInputs, numOutputs, directFeedthroughFlag, timeInvariantFlag);

      obj = obj.setInputFrame(compassGaitPlant.getOutputFrame);
      obj = obj.setOutputFrame(compassGaitPlant.getInputFrame);
      obj = setSimulinkParam(obj,'InitialStep','1e-3','MaxStep','0.05'); 

      % some bookkeeping stuff
      obj.compassGaitPlant = compassGaitPlant;
      obj.numControlInputs = compassGaitPlant.getNumInputs;
      obj.numContStates = compassGaitPlant.getNumContStates;
      obj.numDiscStates = compassGaitPlant.getNumDiscStates;
      obj.numStates = compassGaitPlant.getNumContStates + compassGaitPlant.getNumDiscStates;


      obj = obj.setupLQRCosts();     
    end

    function obj = setupLQRCosts(obj)
      obj.Q = 10*eye(obj.numContStates);
      obj.R = 1*eye(obj.numControlInputs);
      obj.Qf = obj.Q;
    end

    function obj = setNominalTrajectory(obj, ytraj)
      obj.ytraj_des = ytraj;
    end

    function obj = setupLQRControllers(obj)
      obj.trajTimes = [0,obj.ytraj_des.te];

      for idx = 1:length(obj.ytraj_des.traj)
        obj = obj.setupSingleLQRController(idx);
      end
    end

    function obj = setupSingleLQRController(obj, idx)
      traj = obj.ytraj_des.traj{idx};
      y0 = traj.eval(0);
      modeIdx = y0(1);
      plantModel = obj.compassGaitPlant.modes{modeIdx};
      xtraj = traj(1+obj.numDiscStates:obj.numStates); % pickup on the cts part of the states
      utraj = traj((1+obj.numStates):(obj.numStates+obj.numControlInputs));
      
      % frame bullshit that we have to worry about
      xtraj = xtraj.setOutputFrame(plantModel.getStateFrame);
      utraj = utraj.setOutputFrame(plantModel.getInputFrame);
      % options = struct('xdtraj', xtraj, 'udtraj', utraj); % I think this was misleading and incorrect
      options = struct();
      obj.lqrControllers{idx} = tvlqr(plantModel,xtraj,utraj,obj.Q, obj.R, obj.Qf, options);
    end

    function y = output(obj,t,x,u)
      y = obj.currentControlInput;
    end

    function obj = step(obj,t,x)
      % disp('controller called')
      obj.currentControlInput = obj.controlInputFromPlantState(t,x);
      obj.tickCounter = obj.tickCounter + 1;
    end

    function u = getCurrentControlInput(obj)
      u = obj.currentControlInput;
    end

    function idx = getCurrentTrajectoryIdx(obj,t)
      tmp = find(obj.trajTimes <= t);
      idx = tmp(end);
    end

    function [idxLowerBound, idxUpperBound] = getCurrentTrajectoryInterval(obj,t)
      tmp = find(obj.trajTimes <= t);
      idxLowerBound = tmp(end);

      idxUpperBound = -1; % should never be returned;
      tmp = find(obj.trajTimes >= t);
      if isempty(tmp)
        idxUpperBound = length(obj.trajTimes);
      else
        idxUpperBound = tmp(1);
      end
    end

    function [t_plan, idx] = getClosestPlanTimeInSameMode(obj, t,x)

      idx = obj.getCurrentTrajectoryIdx(t);
      sensedMode = x(1);
      tmp = obj.ytraj_des.eval(t);
      planMode = tmp(1);


      % if the sensed and planned modes agree then everything is fine, there is nothing to do
      if (sensedMode == planMode)
        t_plan = t;
        return;
      end

      [idxLowerBound, idxUpperBound] = obj.getCurrentTrajectoryInterval(t);
      distanceToLower = abs(obj.trajTimes(idxLowerBound) - t);
      distanceToUpper = abs(obj.trajTimes(idxUpperBound) - t);

      % initialize variables
      t_plan = -1;
      idx = -1;
      tGuard = -1;

      if (distanceToLower <= distanceToUpper)
        % in this case we entered the new hybrid mode early, so go back to the previous one
        tGuard = obj.trajTimes(idxLowerBound);
        idx = idxLowerBound;
      else
        tGuard = obj.trajTimes(idxUpperBound);
        idx = idxUpperBound;
      end

      tGuardPlus = obj.clipToPlanTimeLimits(tGuard + eps);
      tGuardMinus = obj.clipToPlanTimeLimits(tGuard - eps);

      tmp = obj.ytraj_des.eval(tGuardPlus);
      modePlus = tmp(1);

      tmp = obj.ytraj_des.eval(tGuardMinus);
      modePlus = tmp(1);

      


      % see if our mode matches that before or after the guard
      if (sensedMode == modePlus)
        t_plan = tGuardPlus;
      else
        t_plan = tGuardMinus;
      end

      % do the adjuste
      disp('sensed and plan mode disagreed')
      t
      t_plan
      sensedMode
      planMode
      
    end

    function xcont = extractCtsStateFromFullState(obj, x)
      xcont = x(1+obj.numDiscStates:obj.numStates);
    end


    function u = controlInputFromPlantState(obj,t,x)
      modeIdx = 0;
      idx = 0;

      if obj.controlBasedOnSensedMode
        modeIdx = x(1);
        [t,idx] = obj.getClosestPlanTimeInSameMode(t,x);
      else % this just means do everything according to the plan . . . 
        yPlan = obj.ytraj_des.eval(t);
        modeIdx = yPlan(1);
        idx = obj.getCurrentTrajectoryIdx(t);
      end

      plantModel = obj.compassGaitPlant.modes{modeIdx};
      controller = obj.lqrControllers{idx};

      xcont = obj.extractCtsStateFromFullState(x);
      xPoint = Point(plantModel.getOutputFrame, xcont);
      xInControllerFrame = xPoint.inFrame(controller.getInputFrame,t).p;

      
      uInControllerFrame = controller.output(t,0,xInControllerFrame);
      uPoint = Point(controller.getOutputFrame, uInControllerFrame);
      uInPlantFrame = uPoint.inFrame(plantModel.getInputFrame, t).p;

      u = uInPlantFrame;
    end

    function t = clipToPlanTimeLimits(obj,t)
      t = CompassGaitController.clip(t,obj.ytraj_des.tspan(1), obj.ytraj_des.tspan(2));
    end
    
  end 

  methods(Static)

    function x = clip(x,minVal,maxVal)
      x = min(x,maxVal);
      x = max(x,minVal);
    end
  end
  
end
