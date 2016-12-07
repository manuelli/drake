classdef EKFParticle < CompassGaitParticle
  properties
    sigma_;
  end

  methods
    function obj = EKFParticle(inputData)
      obj = obj@CompassGaitParticle(inputData);
      obj.sigma_ = inputData.sigma;
    end
  end

end