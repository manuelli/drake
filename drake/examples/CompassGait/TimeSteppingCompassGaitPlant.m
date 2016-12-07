classdef TimeSteppingCompassGaitPlant < CompassGaitPlant
  properties
    dataHandle
    odefun;
    eventFun;
    odeOptions;
    cgUtils_
    stepSize_ = 0.002;
  end
  methods
    function obj = TimeSteppingCompassGaitPlant(gammaIn)
      obj = obj@CompassGaitPlant(gammaIn);
      obj.dataHandle = SharedDataHandle(struct());
      obj.dataHandle.data.u = 0;
      obj = obj.initializeDynamicsAndEventFunctions();
      obj.cgUtils_ = CompassGaitUtils();
    end

    function obj = initializeDynamicsAndEventFunctions(obj)
      obj.odefun = @(t,x) obj.dynamics(t,[1;x],0);
      obj.eventFun = @(t,x) obj.eventFunction(t,x,0);
      obj.odeOptions = odeset('Events', obj.eventFun);
    end

    function [ytraj, xtraj] = simulateWithConstantControlInput(obj, x_initial, tspan, u)
      obj.dataHandle.data.u = u;
      [ytraj, xtraj] = simulate(obj, tspan, x_initial);
    end
    
    % same as above but using ode45 instead of Simulink.
    % this is basically the inner loop
    function [t,y, te, ye, ie] = simulateWithConstantControlInputODE(obj, x_initial, tspan, u)

      % x_initial should be of size 4;
      
      obj.dataHandle.data.u = u;      
      [t,y, te, ye, ie] = ode45(obj.odefun, tspan, x_initial, obj.odeOptions);            
    end

    function [x_final] = simulateWithConstantControlInputODE4IgnoreHybridGuard(obj, x_initial, tspan, u, processNoiseLocal, dt)

      if nargin < 6
        dt = obj.stepSize_;
      else
        dt = options.dt;
      end

      odeFunStochastic = @(t,x) obj.dynamics(t,[1;x],0) + processNoiseLocal;
      % x_initial should be of size 4;
      obj.dataHandle.data.u = u;
      numSteps = max(ceil((tspan(2) - tspan(1))/dt),2);
      tspanFull = linspace(tspan(1),tspan(2),numSteps);
      
      obj.dataHandle.data.u = u;      
      [t,y] = ode4(odeFunStochastic, tspanFull, x_initial);

      yPrime = y';
      x_final = yPrime(:,end);
    end

    % do some extra checking to see if we have changed modes, if so then run ode45
    function [t, y, hybridEvent] = simulateWithConstantControlInputODE4(obj, x_initial, tspan, u, processNoiseLocal, dt)

      if nargin < 6
        dt = obj.stepSize_;
      end

      odeFunStochastic = @(t,x) obj.dynamics(t,[1;x],0) + processNoiseLocal;

      hybridEvent = false;

      % x_initial should be of size 4;
      obj.dataHandle.data.u = u;
      numSteps = max(ceil((tspan(2) - tspan(1))/dt),2);
      tspanFull = linspace(tspan(1),tspan(2),numSteps);
      
      obj.dataHandle.data.u = u;      
      [t,y] = ode4(odeFunStochastic, tspanFull, x_initial);

      % check if we had a zero crossing and do some extra work.
      yPrime = y';
      x_final = yPrime(:,end);
      guardVal = obj.eventFunction(0,x_final,0);

      % this means we had a zero crossing, so redo the work using ode45
      if (guardVal < 0)
        [t,y, te, ye, ie] = ode45(odeFunStochastic, tspan, x_initial, obj.odeOptions);
        hybridEvent = true;
        disp('falling back to ode45')
      end
    end


    function [value, isTerminal, direction] = eventFunction(obj, t, x, u)
      g1 = obj.footCollisionGuard1(t,x,u);
      g2 = obj.footCollisionGuard2(t,x,u);

      value = max(g1, g2);
      isTerminal = true;
      direction  = -1; % only record events crossing from positive to negative
    end

    function xcdot = dynamics(obj,t,x,u)
      uController = obj.dataHandle.data.u; % the stored control input for this short simulation
      xcdot = dynamics@CompassGaitPlant(obj, t, x, uController);
    end

    % processNoise should be of the same dimension as xcdot
    % should be specified once at beginning of the integration
    % the processNoise should be in local coords
    function [t,y, te, ye, ie] = simulateWithConstantControlInputODEStochastic(obj, x_initial, tspan, u, processNoiseLocal)

      obj.dataHandle.data.u = u;
      odeFunStochastic = @(t,x) obj.dynamics(t,[1;x],0) + processNoiseLocal;
      [t,y, te, ye, ie] = ode45(odeFunStochastic, tspan, x_initial, obj.odeOptions);
    end


    % basically the same as simulateWithConstantControlInputODE but it doesn't just 
    % return when it hits the hybrid guard, it simulates through it.
    % Everything is in LOCAL coordinates, (theta_swing, theta_stance)
    % x_initial is in local coordinates already
    % only need the hybrid mode for mapping the control input from global --> local
    % x_final is in local coordinates
    function [x_final, hybridSwitch, finalHybridMode, outputData] = simulateThroughHybridEvent(obj,hybridMode, x_initial, tspan, uGlobal, processNoiseGlobal)

      if (nargin < 6)
        processNoiseGlobal = zeros(4,1);
      end

      % just placeholder for data about the hybrid event
      % fields: hybridEventTime, xMinus, xPlus. xMinus_mode, xPlus_mode
      outputData = struct(); 

      uLocal = obj.cgUtils_.transformGlobalControlToLocalControl(hybridMode, uGlobal);

      % probably don't need to do this, variance for left and right will be the same
      % processNoiseLocal = obj.cgUtils.transformVectorGlobalToLocal(hyrbidMode, processNoiseGlobal);


      % [t,y, te, ye, ie] = obj.simulateWithConstantControlInputODEStochastic(x_initial,tspan,uLocal, processNoiseGlobal); 

      [t,y,hybridSwitch] = obj.simulateWithConstantControlInputODE4(x_initial,tspan,uLocal, processNoiseGlobal);


      yPrime = y';
      x_final = yPrime(:,end);
      finalHybridMode = hybridMode;

      % figure out if we need to simulate again because of hitting the hybrid guard
      if( hybridSwitch > 0)
        hybridSwitch = true;
        finalHybridMode = obj.cgUtils_.getOtherHybridMode(hybridMode);

        uLocal = obj.cgUtils_.transformGlobalControlToLocalControl(finalHybridMode, uGlobal);
        [xp, ~] = obj.collisionDynamics(1, 0, x_final);
        t_hybrid_guard = t(end);
        tspan_post_impact = [t_hybrid_guard, tspan(2)];

        % record some debugging info
        outputData.xMinus_mode = hybridMode;
        outputData.xPlus_mode = finalHybridMode;
        outputData.xMinus = x_final; % this is xm
        outputData.xPlus = xp;
        outputData.hybridEventTime = t_hybrid_guard;

        % [t,y,te,ye,ie] = obj.simulateWithConstantControlInputODEStochastic(xp, tspan_post_impact, uLocal, processNoiseGlobal);

        [t,y,~] = obj.simulateWithConstantControlInputODE4(xp, tspan_post_impact, uLocal, processNoiseGlobal);

        yPrime = y';
        x_final = yPrime(:,end);
      end
    end


  end
end