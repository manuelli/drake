classdef CompassGaitParticleFilter < handle
  properties
    plant_; % should be a timestepping compass gait plant
    particleSet_ = {};
    options_;
    cgUtils_;
    tPrev_; % the last time
    t_; % current measurement update time, so we know how far to propagate the particles
    nominalTraj_;
    measurementNoiseCovarianceMatrix_;
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
      defaultOptions.measurementNoiseIMUStdDev = 0.005;
      defaultOptions.measurementNoiseEncodersStdDev = 0.001;

      defaultOptions.initializationPositionNoiseStdDev = 0.02;
      defaultOptions.initializationVelocityNoiseStdDev = 0.05;


      defaultOptions.numTruthSpawningParticles = 5;
      defaultOptions.truthParticlesStdDev_q = 0;
      defaultOptions.truthParticlesStdDev_v = 0;

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

        obj.measurementNoiseCovarianceMatrix_ = ones(2,2)*obj.options_.measurementNoiseIMUStdDev + eye(2)*obj.options_.measurementNoiseEncodersStdDev;
      end
    end

    function plotParticleSet(obj, particleSet, figHandle, options)
      if (nargin < 3)
        figHandle = figure();
      end

      if (nargin < 4)
        options = struct();
      end

      figure(figHandle);

      hold on;

      fnplt(obj.nominalTraj_, [1,3]);
      fnplt(obj.nominalTraj_, [2,4]);


      for i=1:numel(particleSet)
        particle = particleSet{i};
        obj.plotSingleParticle(particle, options);
        % colorString = obj.getParticlePlotColor(particle);
        % scatter(particle.x_.qL, particle.x_.vL, colorString, 'filled');
      end
    end

    function plotSingleParticle(obj, particle, options)
      if nargin < 3
        options = struct();
      end

      if isfield(options, 'colorString')
        colorString = options.colorString;
      else
        colorString = obj.getParticlePlotColor(particle);
      end

      if isfield(options, 'plotRightLeg')
        if (options.plotRightLeg)
          scatter(particle.x_.qR, particle.x_.vR, colorString, 'filled');
          return;
        end
      else
        scatter(particle.x_.qL, particle.x_.vL, colorString, 'filled');
      end

      
    end

    function plotParticleSetWeights(obj,particleSet)
      weightVector = [];
      for i=1:numel(particleSet)
        weightVector(end+1) = particleSet{i}.importanceWeight_;
      end

      weightVector = weightVector/sum(weightVector);
      bar(weightVector);


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
    function applyMeasurementUpdate(obj, y, dt)
      for i=1:numel(obj.particleSet_)
        particle = obj.particleSet_{i};
        obj.applyMeasurementUpdateSingleParticle(particle, y, dt);
      end
    end

    % helps to keep the filter from diverging or 'getting lost'
    function addTruthParticles(obj, trueParticle, options)
      for i=1:options.numParticles
        obj.particleSet_{end+1} = CompassGaitParticle.copy(trueParticle);
      end
    end

    function applyImportanceResampling(obj, options)

      if nargin < 2
        options = struct();
        spawnTruthParticles = false;
        numTruthParticles = 0;
      else
        spawnTruthParticles = true;
        numTruthParticles = options.numTruthParticles;
      end

      weightVector = zeros(numel(obj.particleSet_),1);
      for i=1:numel(obj.particleSet_)
        particle = obj.particleSet_{i};
        weightVector(i) = particle.importanceWeight_;
      end

      weightVector = weightVector/sum(weightVector); % normalize to sum to 1;
      cdfVec = [0;cumsum(weightVector)];

      N_vals = obj.options_.numParticles;
      rand_vals = rand(N_vals,1);

      idxVec = 0:numel(obj.particleSet_);
      out_val = interp1(cdfVec, idxVec, rand_vals);


      % does the resampling
      newParticleSet = {};
      for i=1:obj.options_.numParticles
        idx = ceil(out_val(i));
        particle = obj.particleSet_{idx};
        newParticle = CompassGaitParticle.copy(particle);
        newParticleSet{end+1} = newParticle;
      end

      oldParticleSet = obj.particleSet_; % maybe want to store this for later . . . 
      obj.particleSet_ = newParticleSet;
    end


    % variance should scale by 1/dt
    function processNoise = getProcessNoiseSample(obj, dt)
      qLNoise = 1/(sqrt(dt))*normrnd(0, obj.options_.processNoiseStdDev_q);
      qRNoise = 1/(sqrt(dt))*normrnd(0, obj.options_.processNoiseStdDev_q);

      vLNoise = 1/(sqrt(dt))*normrnd(0, obj.options_.processNoiseStdDev_v);
      vRNoise = 1/(sqrt(dt))*normrnd(0, obj.options_.processNoiseStdDev_v);


      processNoise = [qLNoise; qRNoise; vLNoise; vRNoise];
    end

    function outputData = applyMotionModelSingleParticle(obj, particle, uGlobal, dt, options)

      if nargin<5
        options = struct();
        options.useUncertainty = true;
      end


      tspan = [0, dt];
      x_initial = obj.cgUtils_.transformGlobalStateToLocalState(particle.hybridMode_, particle.x_);

      processNoiseGlobal = obj.getProcessNoiseSample(dt);

      % allow the option of not applying process noise
      if (~options.useUncertainty)
        processNoiseGlobal = 0*processNoiseGlobal;
      end

      [x_final, hybridSwitch, finalHybridMode, outputData] = obj.plant_.simulateThroughHybridEvent(particle.hybridMode_, x_initial, tspan, uGlobal, processNoiseGlobal);

      x_final_global = obj.cgUtils_.transformLocalStateToGlobalState(finalHybridMode, x_final);
      particle.x_ = x_final_global;
      particle.hybridMode_ = finalHybridMode;
    end


    % observation variance should scale by 1/dt, so stdDev goes like 1/sqrt(dt)
    function applyMeasurementUpdateSingleParticle(obj, particle, y, dt)
      y_hat = [particle.x_.qL, particle.x_.qR];
      y = reshape(y,[1,2]);
      particle.importanceWeight_ = mvnpdf(y_hat, y, obj.measurementNoiseCovarianceMatrix_/sqrt(dt));



%       if (isnan(particle.importanceWeight_) || isinf(particle.importanceWeight_) || (particle.importanceWeight_ < 1e-5))
%         temp = 0;
%       end
       % normpdf(y_hat(1),y(1), obj.options_.measurementNoiseStdDev/sqrt(dt)) * normpdf(y_hat(2),y(2), obj.options_.measurementNoiseStdDev/sqrt(dt));
      
    end

    % observation variance should scale by 1/dt, so stdDev goes like 1/sqrt(dt)
    function y = generateObservation(obj, particle, dt)
      mu = [particle.x_.qL;particle.x_.qR];
      y = mvnrnd(mu, obj.measurementNoiseCovarianceMatrix_/sqrt(dt));
    end

    function particleSetCopy = getCopyOfParticleSet(obj)
      particleSetCopy = CompassGaitParticle.copyParticleSet(obj.particleSet_);
    end
  end

end