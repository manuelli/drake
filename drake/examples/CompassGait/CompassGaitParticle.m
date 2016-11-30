% Just a container class for storing relevant fields for a particle of the compass gait
classdef CompassGaitParticle < handle
  properties
    qL_;
    qR_;
    vL_;
    vR_;
    x_; % stores the state, has fields qL, qR, vL, vR
    hybridMode_; % can be 1 or 2, 1 for left leg swing, 2 for right left swing
    importanceWeight_;
    options_;
    dt_; % timestep of the simulation, needed for computing variance of measurement noise
  end

  methods
    function obj = CompassGaitParticle(inputData)
      obj.x_ = struct();
      obj.x_= inputData.xGlobal;
      obj.hybridMode_ = inputData.hybridMode;
    end
  end

end