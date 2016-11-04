classdef TimeSteppingCompassGaitPlant < CompassGaitPlant
  properties
    dataHandle
  end
  methods
    function obj = TimeSteppingCompassGaitPlant(gammaIn)
      obj = obj@CompassGaitPlant(gammaIn);
      dataHandle = SharedDataHandle(struct());
    end

    function [ytraj, xtraj] = simulateWithConstantControlInput(obj, x_initial, tspan, u)
      obj.dataHandle.data.u = u;
      [ytraj, xtraj] = simulate(obj, tspan, x_initial);
    end

    function xcdot = dynamics(obj,t,x,u)
      uController = obj.dataHandle.data.u; % the stored control input for this short simulation
      xcdot = dynamics@CompassGaitPlant(obj, t, x, uController);
    end
  end

end