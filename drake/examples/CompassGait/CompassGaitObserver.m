classdef CompassGaitObserver < handle

	properties
		options_;
		plant_;
		observerGain_;
		observationMatrix_;
		xLocal_;
		hybridMode_;
		cgUtils_;
		zeroProcessNoiseLocal_;
    lastResetTime_ = -100; % default initialization, so that no it looks like no reset has happened recently
    name_ = '';
	end

	methods
		function obj = CompassGaitObserver(plant, options)
			if nargin < 2
				options = struct();
			end
			obj.plant_ = plant;
			obj.cgUtils_ = CompassGaitUtils();
			obj.initializeOptions(options);
			obj.zeroProcessNoiseLocal_ = zeros(4,1);
			obj.observationMatrix_ = [eye(2),zeros(2,2)];
			obj.setupObserverGainMatrix();
		end

		function initializeOptions(obj, options)
			defaultOptions = struct();
			defaultOptions.epsilon = 0.2;

			obj.options_ = applyDefaults(options, defaultOptions);
		end

		function initialize(obj, hybridMode, xGlobal)
			obj.hybridMode_ = hybridMode;
			obj.xLocal_ = obj.cgUtils_.transformGlobalStateToLocalState(hybridMode, xGlobal);
		end

		function setupObserverGainMatrix(obj)
			epsilon = obj.options_.epsilon;
			obj.observerGain_ = [2.0/epsilon*eye(2); 1.0/epsilon^2*eye(2)];
		end

		function applyMotionModel(obj, uGlobal, dt)
			tspan = [0, dt];
			uLocal = obj.cgUtils_.transformGlobalControlToLocalControl(obj.hybridMode_, uGlobal);

			% propagate the model forward without using the hybrid guard stuff
			obj.xLocal_ = obj.plant_.simulateWithConstantControlInputODE4IgnoreHybridGuard(obj.xLocal_, tspan, uLocal, obj.zeroProcessNoiseLocal_);
		end

    function hybridSwitch = applyMotionModelWithHybridGuard(obj, uGlobal, dt)
      tspan = [0, dt];
      uLocal = obj.cgUtils_.transformGlobalControlToLocalControl(obj.hybridMode_, uGlobal);

      % propagate the model forward without using the hybrid guard stuff
      % now propagate dynamics forwards, ignoring hybrid guard
      processNoiseGlobal = zeros(4,1); % just want pure forward sim
      tspan = [0,dt];

      [x_final, hybridSwitch, finalHybridMode, outputData] = obj.plant_.simulateThroughHybridEvent(obj.hybridMode_, obj.xLocal_, tspan, uGlobal, processNoiseGlobal);

      obj.xLocal_ = x_final;
      obj.hybridMode_ = finalHybridMode;

      if hybridSwitch
        disp('observer hit hybrid guard');
      end
    end

		% assume observations of both positions
		% dt is how long it's been since the last observation
		function applyMeasurementUpdate(obj, yGlobal, dt)
			yLocal = obj.cgUtils_.transformObservationGlobalToLocal(obj.hybridMode_, yGlobal);
      yLocal = reshape(yLocal, 2, 1);
			yLocal_hat = obj.observationMatrix_*obj.xLocal_;
			obj.xLocal_ = obj.xLocal_ + dt*obj.observerGain_*(yLocal - yLocal_hat);
		end

		function applyResetMap(obj)
      t = 0;
      u = 0;
      [xp, nextMode] = obj.plant_.collisionDynamics(obj.hybridMode_, t, obj.xLocal_, u);
      obj.xLocal_ = xp;
      obj.hybridMode_ = nextMode;
    end

		function particle = getObserverStateAsParticle(obj)
      inputData = struct();
      xGlobal = obj.cgUtils_.transformLocalStateToGlobalState(obj.hybridMode_, obj.xLocal_);

      inputData.hybridMode = obj.hybridMode_;
      inputData.xGlobal = xGlobal;

      particle = CompassGaitParticle(inputData);
    end

	end

end