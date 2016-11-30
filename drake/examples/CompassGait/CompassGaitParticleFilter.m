classdef CompassGaitParticleFilter < handle
  properties
    plant_; % should be a timestepping compass gait plant
    particleSet_ = {};
    options_;
    cgUtils_;
    tPrev_; % the last time
    t_; % current measurement update time, so we know how far to propagate the particles
    nominalTraj_;
  end

  methods
    function obj = CompassGaitParticleFilter(plant,options)

      if (nargin < 2)
        options = struct();
      end

      obj.plant_ = plant;
      obj = obj.initializeOptions(options);
      obj.cgUtils_ = CompassGaitUtils();
      obj.t_ = 0;
      obj.tPrev_ = 0;
    end

    function obj = initializeOptions(obj, options)
      defaultOptions = struct();
      defaultOptions.numParticles = 5;
      defaultOptions.processNoiseStdDev_q = 0;
      defaultOptions.processNoiseStdDev_v = 0.05; % these should be related by a square or something
      defaultOptions.measurementNoiseStdDev = 1;

      defaultOptions.initializationPositionNoiseStdDev = 0.02;
      defaultOptions.initializationVelocityNoiseStdDev = 0.05;

      obj.options_ = applyDefaults(options, defaultOptions);
    end

    % say where to start the particles
    function obj = initializeFilter(obj, hybridMode, xGlobal)
      obj.particleSet_ = {}; % clear any existing particle set
      for i=1:obj.options_.numParticles
        inputData = struct();
        inputData.hybridMode = hybridMode;
        xGlobalNew = struct();


        xGlobalNew = struct();
        xGlobalNew.qL = xGlobal.qL + normrnd(0,obj.options_.initializationPositionNoiseStdDev);
        xGlobalNew.qR = xGlobal.qR + normrnd(0,obj.options_.initializationPositionNoiseStdDev);

        xGlobalNew.vL = xGlobal.vL + normrnd(0,obj.options_.initializationPositionNoiseStdDev);
        xGlobalNew.vR = xGlobal.vR + normrnd(0,obj.options_.initializationPositionNoiseStdDev);

        inputData.xGlobal = xGlobalNew;

        obj.particleSet_{end+1} = CompassGaitParticle(inputData);
      end
    end

    function plotParticleSet(obj, particleSet, figHandle)
      if (nargin < 3)
        figHandle = figure();
      end

      figure(figHandle);

      hold on;

      fnplt(obj.nominalTraj_, [1,3]);
      fnplt(obj.nominalTraj_, [2,4]);


      for i=1:numel(particleSet)
        particle = particleSet{i};
        colorString = obj.getParticlePlotColor(particle);
        scatter(particle.x_.qL, particle.x_.vL, colorString, 'filled');
      end
    end

    function colorString = getParticlePlotColor(obj, particle)
      if(particle.hybridMode_ == 1)
        colorString = 'r';
      else
        colorString = 'b';
      end
    end

    % propagate forwards by a given amount
    function applyMotionModel(obj, uGlobal, dt)
      for i=1:numel(obj.particleSet_)
        particle = obj.particleSet_{i};
        obj.applyMotionModelSingleParticle(particle, uGlobal, dt);
      end
    end

    % y is the observation, should have qL and qR fields
    function applyMeasurementModel(obj, y)

    end

    function applyImportanceResampling(obj)

    end

    function processNoise = getProcessNoiseSample(obj, dt)
      qLNoise = 1/(sqrt(dt))*normrnd(0, obj.options_.processNoiseStdDev_q);
      qRNoise = 1/(sqrt(dt))*normrnd(0, obj.options_.processNoiseStdDev_q);

      vLNoise = 1/(sqrt(dt))*normrnd(0, obj.options_.processNoiseStdDev_v);
      vRNoise = 1/(sqrt(dt))*normrnd(0, obj.options_.processNoiseStdDev_v);


      processNoise = [qLNoise; qRNoise; vLNoise; vRNoise];
    end

    function applyMotionModelSingleParticle(obj, particle, uGlobal, dt)
      tspan = [0, dt];
      x_initial = obj.cgUtils_.transformGlobalStateToLocalState(particle.hybridMode_, particle.x_);

      processNoiseGlobal = obj.getProcessNoiseSample(dt);

      [x_final, hybridSwitch, finalHybridMode] = obj.plant_.simulateThroughHybridEvent(particle.hybridMode_, x_initial, tspan, uGlobal, processNoiseGlobal);

      x_final_global = obj.cgUtils_.transformLocalStateToGlobalState(finalHybridMode, x_final);
      particle.x_ = x_final_global;
      particle.hybridMode_ = finalHybridMode;
    end

    function applyMeasurementUpdateSingleParticle(obj, particle, y)

    end
  end

end