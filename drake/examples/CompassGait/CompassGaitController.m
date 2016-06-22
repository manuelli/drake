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

    function xcont = extractCtsStateFromFullState(obj, x)
      xcont = x(1+obj.numDiscStates:obj.numStates);
    end


    function u = controlInputFromPlantState(obj,t,x)
      modeIdx = x(1);
      idx = obj.getCurrentTrajectoryIdx(t);
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


    
  end 
  
end
