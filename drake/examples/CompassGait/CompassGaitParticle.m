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

  methods (Static)
    function newParticle = copy(particle)
      inputData = struct();
      inputData.hybridMode = particle.hybridMode_;
      inputData.xGlobal = particle.x_;
      newParticle = CompassGaitParticle(inputData);
      newParticle.importanceWeight_ = particle.importanceWeight_;
    end

    function newParticleSet = copyParticleSet(particleSet)
      newParticleSet = {};

      for i=1:numel(particleSet)
        newParticleSet{i} = CompassGaitParticle.copy(particleSet{i});
      end
    end


    function sameMode = allParticlesInSameMode(particleSet)
      sameMode = true;
      hybridMode = particleSet{1}.hybridMode_;
      for i=1:numel(particleSet)
        if (particleSet{i}.hybridMode_ ~= hybridMode)
          sameMode = false;
          return;
        end
      end
    end

    function [modeParticleSets ] = sortParticlesIntoModes(particleSet)
      modeParticleSets = struct();
      mode1 = {};
      mode2 = {};
      for i=1:numel(particleSet)
        particle = particleSet{i};
        if (particle.hybridMode_ == 1)
          mode1{end+1} = CompassGaitParticle.copy(particle);
        else
          mode2{end+1} = CompassGaitParticle.copy(particle);
        end
      end

      modeParticleSets.mode1 = mode1;
      modeParticleSets.mode2 = mode2;
    end


    % this probably only makes sense if all the particles are in the same mode
    function avgParticle = avgParticleSet(particleSet)
      cgUtils = CompassGaitUtils();
      xtemp = zeros(4,1);
      numParticles = numel(particleSet); 
      for i=1:numParticles
        p = particleSet{i};
        xtemp = xtemp + cgUtils.transformGlobalStateToLocalState(p.hybridMode_, p.x_);
      end

      xAvg = xtemp/numParticles;
      xAvgGlobal = cgUtils.transformLocalStateToGlobalState(p.hybridMode_, xAvg);

      inputData.hybridMode = p.hybridMode_;
      inputData.xGlobal = xAvgGlobal;
      avgParticle = CompassGaitParticle(inputData);
    end

    function [returnData] = getAvgParticleInEachMode(particleSets)
      returnData = CompassGaitParticle.sortParticlesIntoModes(particleSets);
      
      if(numel(returnData.mode1) > 0)
        returnData.mode1_avg = CompassGaitParticle.avgParticleSet(returnData.mode1);
      end

      if(numel(returnData.mode2) > 0)
        returnData.mode2_avg = CompassGaitParticle.avgParticleSet(returnData.mode2);
      end
    end


  end

end