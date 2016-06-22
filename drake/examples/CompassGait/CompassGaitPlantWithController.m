classdef CompassGaitPlantWithController < CompassGaitPlant
  properties
    controller
  end
  
  methods 
    function obj = CompassGaitPlantWithController()
      obj = obj@CompassGaitPlant()
    end

    function xdn = update(obj,t,x,u_in)
      obj.controller = obj.controller.step(t,x);
      u_controller = obj.controller.getCurrentControlInput();
      xdn = update@CompassGaitPlant(obj,t,x,u_controller);
    end

    function xcdot = dynamics(obj,t,x,u_in)
      obj.controller = obj.controller.step(t,x);
      u_controller = obj.controller.getCurrentControlInput();
      xcdot = dynamics@CompassGaitPlant(obj,t,x,u_controller);
    end

    function y = output(obj,t,x,u_in)
      % just a hack for now, redo this later . . . ?
      obj.controller = obj.controller.step(t,x);
      u_controller = obj.controller.getCurrentControlInput();
      y = output@CompassGaitPlant(obj,t,x,u_controller);
    end

    function obj = setController(obj, controller)
      obj.controller = controller;
    end 
  end
  
  
end
