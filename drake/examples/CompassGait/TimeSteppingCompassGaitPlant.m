classdef TimeSteppingCompassGaitPlant < CompassGaitPlant
  properties
    dataHandle
    odefun;
    eventFun;
    odeOptions;
  end
  methods
    function obj = TimeSteppingCompassGaitPlant(gammaIn)
      obj = obj@CompassGaitPlant(gammaIn);
      obj.dataHandle = SharedDataHandle(struct());
      obj.dataHandle.data.u = 0;
      obj = obj.initializeDynamicsAndEventFunctions();
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
      tic;
      obj.dataHandle.data.u = u;      
      [t,y, te, ye, ie] = ode45(obj.odefun, tspan, x_initial, obj.odeOptions);
      toc      
    end


    function simulateODE(obj, x_initial_full, tspan)

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
  end
end