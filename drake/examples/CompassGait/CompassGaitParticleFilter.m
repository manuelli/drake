classdef CompassGaitParticleFilter < handle
  properties
    plant_; % should be a timestepping compass gait plant
    particleSet_ = {};
    options_;
    cgUtils_;
    tPrev_; % the last time
    t_; % current measurement update time, so we know how far to propagate the particles
    nominalTraj_;
    % measurementNoiseCovarianceMatrix_;
    measurementNoiseCovarianceMatrixUnscaled_;
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
      defaultOptions.measurementNoiseIMUVar = 0.005;
      defaultOptions.measurementNoiseEncodersVar = 0.001;

      defaultOptions.initializationPositionNoiseStdDev = 0.02;
      defaultOptions.initializationVelocityNoiseStdDev = 0.05;


      defaultOptions.numTruthSpawningParticles = 5;
      defaultOptions.truthParticlesStdDev_q = 0;
      defaultOptions.truthParticlesStdDev_v = 0;

      obj.options_ = applyDefaults(options, defaultOptions);

      % note this is really covariance matrix
      obj.measurementNoiseCovarianceMatrixUnscaled_ = ones(2,2)*obj.options_.measurementNoiseIMUVar+ eye(2)*obj.options_.measurementNoiseEncodersVar;
    end

    % say where to start the particles
    % xGlobal is just a vector here
    function returnData = initializeFilter(obj, hybridMode, xGlobal, options)
      returnData = struct();
      returnData.hitHybridGuard = false;
      if nargin < 4
        options = struct();
        options.sigmaGlobal = diag([obj.options_.initializationPositionNoiseStdDev^2, obj.options_.initializationPositionNoiseStdDev^2, obj.options_.initializationVelocityNoiseStdDev^2, obj.options_.initializationVelocityNoiseStdDev^2 ]);
      end

      xLocal = obj.cgUtils_.transformGlobalStateToLocalState(hybridMode, xGlobal);
      sigmaLocal = obj.cgUtils_.transformGlobalToLocalCovarianceMatrix(hybridMode, options.sigmaGlobal);

      obj.particleSet_ = {}; % clear any existing particle set

      % discard any particles on wrong side of the guard
      counter = 0;
      while(counter < obj.options_.numParticles)
        inputData = struct();
        inputData.hybridMode = hybridMode;

        xLocalNew = mvnrnd(xLocal, sigmaLocal);

        % check that this particle isn't on wrong side of hybrid guard
        if (obj.plant_.eventFunction(0,xLocalNew,0) < 0)
          % disp('sampled a point on wrong side of guard, trying again')
          % counter
          returnData.hitHybridGuard = true;
          continue;
        end

        xGlobalNew = obj.cgUtils_.transformLocalStateToGlobalState(hybridMode, xLocalNew);
        % xGlobalNew.qL = xGlobal.qL + normrnd(0,obj.options_.initializationPositionNoiseStdDev);
        % xGlobalNew.qR = xGlobal.qR + normrnd(0,obj.options_.initializationPositionNoiseStdDev);

        % xGlobalNew.vL = xGlobal.vL + normrnd(0,obj.options_.initializationVelocityNoiseStdDev);
        % xGlobalNew.vR = xGlobal.vR + normrnd(0,obj.options_.initializationVelocityNoiseStdDev);

        inputData.xGlobal = xGlobalNew;

        obj.particleSet_{end+1} = CompassGaitParticle(inputData);
        counter = counter + 1;        
      end
      % note this is really covariance matrix
        obj.measurementNoiseCovarianceMatrixUnscaled_ = ones(2,2)*obj.options_.measurementNoiseIMUVar+ eye(2)*obj.options_.measurementNoiseEncodersVar;
    end

    function plotParticleSet(obj, particleSet, figHandle, options)
      if (nargin < 3)
        figHandle = figure();
      end

      if (nargin < 4)
        options = struct();
      end

      defaultOptions = struct();
      defaultOptions.plotType = 'left';
      defaultOptions.nominalPlotType = 'normal';
      options = applyDefaults(options, defaultOptions);

      figure(figHandle);

      hold on;

      % fnplt(obj.nominalTraj_, [1,3]);
      % fnplt(obj.nominalTraj_, [2,4]);
      obj.plotNominalTraj(options);

      for i=1:numel(particleSet)
        particle = particleSet{i};
        obj.plotSingleParticle(particle, options);
        % colorString = obj.getParticlePlotColor(particle);
        % scatter(particle.x_.qL, particle.x_.vL, colorString, 'filled');
      end
    end

    function plotNominalTraj(obj, options)
      if nargin < 2
        options = struct();
      end

      defaultOptions.nominalPlotType = 'normal';

      options = applyDefaults(options, defaultOptions);


      if (strcmp(options.nominalPlotType, 'normal'))
        fnplt(obj.nominalTraj_, [1,3]);
        fnplt(obj.nominalTraj_, [2,4]);
      else
        fnplt(obj.nominalTraj_, [1, 2]);
        fnplt(obj.nominalTraj_, [2, 1]);
      end
        
    end

    function plotSingleParticle(obj, particle, options)
      if nargin < 3
        options = struct();
      end

      defaultOptions = struct();
      defaultOptions.plotType = 'left';
      options = applyDefaults(options, defaultOptions);

      if isfield(options, 'colorString')
        colorString = options.colorString;
      else
        colorString = obj.getParticlePlotColor(particle);
      end

      if (strcmp(options.plotType, 'right'))
        scatter(particle.x_.qR, particle.x_.vR, colorString, 'filled');
      elseif (strcmp(options.plotType, 'left'))
        scatter(particle.x_.qL, particle.x_.vL, colorString, 'filled');
      elseif (strcmp(options.plotType, 'position'))
        scatter(particle.x_.qL, particle.x_.qR, colorString, 'filled');
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
    function applyMotionModel(obj, uGlobal, dt, options)
      if nargin < 4
        options = struct();
      end
      defaultOptions = struct();
      defaultOptions.useUncertainty = true;

      options = applyDefaults(options, defaultOptions);

      for i=1:numel(obj.particleSet_)
        particle = obj.particleSet_{i};
        obj.applyMotionModelSingleParticle(particle, uGlobal, dt, options);
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
      particle.importanceWeight_ = mvnpdf(y_hat, y, obj.getMeasurementNoiseCovarianceMatrix(dt));      
    end

    % observation variance should scale by 1/dt
    function cov = getMeasurementNoiseCovarianceMatrix(obj, dt)
      cov = 1/dt*obj.measurementNoiseCovarianceMatrixUnscaled_;
    end

    
    function [y, yParticle] = generateObservation(obj, particle, dt, options)
      if nargin < 4
        options = struct();
      end

      defaultOptions = struct();
      defaultOptions.addNoise = true;
      options = applyDefaults(options, defaultOptions);

      mu = [particle.x_.qL,particle.x_.qR];
      covMatrix = obj.getMeasurementNoiseCovarianceMatrix(dt);
      y = mvnrnd(mu, covMatrix);

      if (~options.addNoise)
        y = mu;
      end

      inputData = struct();
      x_ = struct();
      x_.qL = y(1);
      x_.qR = y(2);
      x_.vL = 0;
      x_.vR = 0;
      inputData.xGlobal = x_;
      inputData.sigma = covMatrix;
      inputData.hybridMode = particle.hybridMode_; % not really relevant for this one
      yParticle = EKFParticle(inputData);
    end

    function particleSetCopy = getCopyOfParticleSet(obj)
      particleSetCopy = CompassGaitParticle.copyParticleSet(obj.particleSet_);
    end
  end

end