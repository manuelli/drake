classdef SimpleController
  
  properties
    compassGaitPlant
    tickCounter = 0;
    currentControlInput = 0;
    dataHandle;
  end
  
  methods
    function obj = SimpleController(compassGaitPlant)
      obj.compassGaitPlant = compassGaitPlant;
      dataHandle = SharedDataHandle(struct());
      dataHandle.data.currentControlInput = 0;
      dataHandle.data.currentPlanTime = 0;
      dataHandle.data.tickCounter = 0;
      dataHandle.data.u = [];
      dataHandle.data.times = [];

      obj.dataHandle = dataHandle;
    end
    
    function obj = step(obj,t,x,u)
      obj.dataHandle.data.tickCounter = obj.dataHandle.data.tickCounter + 1;
      obj.dataHandle.data.times = [obj.dataHandle.data.times;t];
    end
    
    function u = getCurrentControlInput(obj)
      u = obj.dataHandle.data.currentControlInput;
    end

    % for use in the time stepping simulation
    function [u, controlData] = tick(obj,t,x)
      u = 0;
      controlData = struct();
    end

    function planTime = getCurrentPlanTime(obj)
      planTime = obj.dataHandle.data.currentPlanTime;
    end
  end
  
end