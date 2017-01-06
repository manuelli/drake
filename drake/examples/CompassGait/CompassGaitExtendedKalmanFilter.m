classdef CompassGaitExtendedKalmanFilter < handle

  properties
    plant_;
    xLocal_;
    hybridMode_;
    sigma_;
    options_;
    cgUtils_;
    xBar_; % xLocal after applying motion model
    sigmaBar_; % variance after applying motion model
    H_; % this is the observation matrix
    kalmanGain_; % the kalman gain
    processNoiseCovarianceMatrixUnscaled_; % unscaled process noise covariance matrix, i.e. it's diagonal
    measurementNoiseCovarianceMatrixUnscaled_;
  end

  methods
    function obj = CompassGaitExtendedKalmanFilter(plant, options)
      obj.plant_ = plant;
      obj.initializeOptions(options);
      obj.cgUtils_ = CompassGaitUtils();
    end

    function obj = initializeOptions(obj, options)
      defaultOptions = struct();
      defaultOptions.numParticles = 5;

      % note these variances are for the cts time thing will need to be adjusted here
      defaultOptions.processNoiseStdDev_q = 0;
      defaultOptions.processNoiseStdDev_v = 0.05; % these should be related by a square or something
      defaultOptions.measurementNoiseIMUVar = 0.005;
      defaultOptions.measurementNoiseEncodersVar = 0.001;

      defaultOptions.initializationPositionNoiseStdDev = 0.02;
      defaultOptions.initializationVelocityNoiseStdDev = 0.05;

      obj.options_ = applyDefaults(options, defaultOptions);
    end

    function initializeFilter(obj, hybridMode, xGlobal)
      obj.xLocal_ = obj.cgUtils_.transformGlobalStateToLocalState(hybridMode, xGlobal);
      obj.hybridMode_ = hybridMode;

      obj.sigma_ = diag([obj.options_.initializationPositionNoiseStdDev^2, obj.options_.initializationPositionNoiseStdDev^2, obj.options_.initializationVelocityNoiseStdDev^2, obj.options_.initializationPositionNoiseStdDev^2]);


      % make a raw covariance matrix for process noise.
      % note that this hasn't been scaled by dt yet
      obj.processNoiseCovarianceMatrixUnscaled_ = diag([obj.options_.processNoiseStdDev_q^2, obj.options_.processNoiseStdDev_q^2, obj.options_.processNoiseStdDev_v^2, obj.options_.processNoiseStdDev_v^2]);

      % this is the observation model
      obj.H_ = [eye(2),zeros(2,2)];

      obj.measurementNoiseCovarianceMatrixUnscaled_ = ones(2,2)*obj.options_.measurementNoiseIMUVar + eye(2)*obj.options_.measurementNoiseEncodersVar;

      % initialize the bar variables so we can immediately apply a measurement update
      obj.xBar_ = obj.xLocal_;
      obj.sigmaBar_ = obj.sigma_;

      % just initialize kalman gain to zero
      % effectively means we will ignore the first measurement
      obj.kalmanGain_ = zeros(4,2);

    end

    function applyMotionModel(obj, uGlobal, dt)
      uLocal = obj.cgUtils_.transformGlobalControlToLocalControl(obj.hybridMode_, uGlobal);

      % the 0 is because we don't care about the time when linearizing
      [A,~] = obj.plant_.modes{1}.linearize(0,obj.xLocal_,uLocal);

      G = A*dt + eye(4);

      % now propagate dynamics forwards, ignoring hybrid guard
      processNoiseLocal = zeros(4,1); % just want pure forward sim
      tspan = [0,dt];
      x_final = obj.plant_.simulateWithConstantControlInputODE4IgnoreHybridGuard(obj.xLocal_, tspan, uLocal, processNoiseLocal);

      processNoiseCovarianceMatrix = obj.getProcessNoiseCovarianceMatrix(dt);
      measurementNoiseCovarianceMatrix = obj.getMeasurementNoiseCovarianceMatrix(dt);

      obj.xBar_ = x_final;
      obj.sigmaBar_ = G*obj.sigma_*G' + processNoiseCovarianceMatrix;

      obj.kalmanGain_ = obj.sigmaBar_*obj.H_'*inv(obj.H_*obj.sigmaBar_*obj.H_' + measurementNoiseCovarianceMatrix);
    end


    % is hybrid mode, the current estimate in the EKF?
    function applyMeasurementUpdate(obj, y_obs_global)
      y = obj.cgUtils_.transformObservationGlobalToLocal(obj.hybridMode_, y_obs_global);
      y = reshape(y,[2,1]);
      yhat = obj.H_*obj.xBar_;

      obj.xLocal_ = obj.xBar_ + obj.kalmanGain_*(y - yhat);
      obj.sigma_ = (eye(4) - obj.kalmanGain_*obj.H_)*obj.sigmaBar_;
    end


    %WARNING: don't think that this is correct, should be 1/dt
    function yhat = generateObservation(obj, x)
      mu = [x(1), x(2)];
      yhat = mvnrnd(mu, obj.measurementNoiseCovarianceMatrix_/sqrt(dt));
    end


    % this only applies the reset map. It doesn't apply the motion model
    function applyResetMap(obj)
      resetMapJacobian = obj.plant_.getResetMapJacobian(obj.xLocal_);

      t = 0;
      u = 0;
      [xp, nextMode] = obj.plant_.collisionDynamics(obj.hybridMode_, t, obj.xLocal_, u);

      sigmaPlus = resetMapJacobian*obj.sigma_*resetMapJacobian';

      obj.sigma_ = sigmaPlus;
      obj.xLocal_ = xp;
      obj.hybridMode_ = nextMode;

      % also make sure you set xBar to this also.
      % otherwise when we apply the measurement update everything will be messed up
      obj.sigmaBar_ = obj.sigma_;
      obj.xBar_ = obj.xLocal_;
    end


    % note that y is in global coordinates
    function kalmanFilterUpdate(obj,uGlobal,dt,y_obs)

    end

    % record the state of the kalman filter.
    % will need a way to visualize these
    function outputData = getKalmanFilterState(obj)
      outputData = struct();
      outputData.xGlobal = obj.xLocal_;
      outputData.hybridMode = obj.hybridMode_;
      outputData.sigma = obj.sigma_;
    end

    % encodes the EKF state as an EKFParticle, makes it convenient to keep track of
    function particle = getKalmanFilterStateAsParticle(obj)
      inputData = struct();
      xGlobal = obj.cgUtils_.transformLocalStateToGlobalState(obj.hybridMode_, obj.xLocal_);
      sigmaGlobal = obj.cgUtils_.transformLocalToGlobalCovarianceMatrix(obj.hybridMode_, obj.sigma_);

      inputData.hybridMode = obj.hybridMode_;
      inputData.xGlobal = xGlobal;
      inputData.sigma = sigmaGlobal;

      particle = EKFParticle(inputData);
    end

    % same as above except that it encodes xBar state, i.e. state after applying motion model
    function particle = getKalmanFilterBarStateAsParticle(obj)
      inputData = struct();
      xGlobal = obj.cgUtils_.transformLocalStateToGlobalState(obj.hybridMode_, obj.xBar_);
      sigmaGlobal = obj.cgUtils_.transformLocalToGlobalCovarianceMatrix(obj.hybridMode_, obj.sigmaBar_);

      inputData.hybridMode = obj.hybridMode_;
      inputData.xGlobal = xGlobal;
      inputData.sigma = sigmaGlobal;

      particle = EKFParticle(inputData);
    end

    function sig = getProcessNoiseCovarianceMatrix(obj,dt)
      % this T term just deals with the fact that velocities get integrated to reach positions
      T = dt*eye(4);
      temp = 1/2.0*dt^2;
      T(1,3) = temp;
      T(2,4) = temp;

      sig = T * (1/dt * obj.processNoiseCovarianceMatrixUnscaled_) * T'; 
    end

    function cov = getMeasurementNoiseCovarianceMatrix(obj,dt)
      cov = 1/dt*obj.measurementNoiseCovarianceMatrixUnscaled_;
    end

    % some plotting stuff
    function plotKalmanFilterState(obj, particle, options)

      if nargin < 3
        options = struct();
      end

      defaultOptions = struct();
      defaultOptions.plotType = 'left';

      options = applyDefaults(options, defaultOptions);

      if (obj.hybridMode_ == 1)
        colorString = 'c'; % cyan for mode 1
      else
        colorString = 'm'; % magenta for mode 2
      end

      idx = [];
      mu = [];

      if (strcmp(options.plotType, 'left'))
        scatter(particle.x_.qL, particle.x_.vL, colorString, 'filled');
        idx = [1,3];
        mu = [particle.x_.qL, particle.x_.vL];
      elseif (strcmp(options.plotType,'right'))
        scatter(particle.x_.qR, particle.x_.vR, colorString, 'filled');
        idx = [2,4];
        mu = [particle.x_.qR, particle.x_.vR];
      elseif (strcmp(options.plotType, 'position'))
        idx = [1,2];
        mu = [particle.x_.qL, particle.x_.qR];
        scatter(particle.x_.qL, particle.x_.qR, colorString, 'filled');
      else
        error('Invalid specicification of options.plotType');
      end

      % plotting the covariance ellipse
      covMatrix = particle.sigma_(idx,idx);
      error_ellipse(covMatrix, mu);
    end


  end

end