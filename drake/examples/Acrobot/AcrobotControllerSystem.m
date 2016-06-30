classdef AcrobotControllerSystem < DrakeSystem

  properties
    acrobotController;
  end

  methods
    function obj = AcrobotControllerSystem(plant, options)
      numInputs = plant.getNumOutputs;
      numOutputs = plant.getNumInputs;
      obj = obj@DrakeSystem(0,0,numInputs,numOutputs);
      if nargin < 2
        options = struct();
      end
      obj.acrobotController = AcrobotController(plant,options);
      obj.setInputFrame(plant.getOutputFrame);
      obj.setOutputFrame(plant.getInputFrame);
    end

    function y = output(obj,t,~,x)
      u = obj.acrobotController.output(t,0,x);
    end

    function xcdot = dynamics(obj,t,x,u)
      xcdot = 0;
    end
  end


end