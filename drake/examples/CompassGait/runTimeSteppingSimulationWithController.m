function returnData = runTimeSteppingSimulationWithController(plant, controller, tspan, dt, x_initial)

  tGrid = [tspan(1):dt:tspan(2)];

  numTimes = length(tGrid);

  xtrajCellArray = {};
  ytrajCellArray = {};
  controlDataCellArray = {};
  uGrid = [];
  hybridEventTimes = [];

  returnData = struct();
  t_current = tspan(1);

  uGrid = [];
  xGrid = [];
  tGrid = [];
  tControlGrid = [];
  currentMode = x_initial(1);
  x_current = x_initial(2:end);


  while(t_current < tspan(2))
    [u, controlData] = controller.tick(t_current,x_current);

    t_start = t_current;
    t_end = t_current + dt;
    [t,y,te,ye,ie] = plant.simulateWithConstantControlInputODE(x_current, [t_start, t_end], u);

    tGrid = [tGrid;t];
    % not quite sure how to record u, yet;
    tControlGrid = [tControlGrid;t_start];
    uGrid = [uGrid;u];

    hybridEventTimes = [hybridEventTimes; te];

    currentSimWithModeInfo = [ones(1,length(t));y'];
    xGrid = [xGrid, currentSimWithModeInfo];

    % book keeping
    yPrime = y';
    x_current = yPrime(:, end);
    t_current = t(end);

    % this means we went through a hybrid reset
    if(length(te) > 0)
      [x_current, currentMode] = plant.collisionDynamics(currentMode, 0, x_current);
    end
  end

  % returnData.utraj = PPTrajectory(pchip(tGrid(1:numTimes-1), uGrid));
  returnData.hybridEventTimes = hybridEventTimes;


  [tGridUnique, uniqueIdx, ~] = unique(tGrid);

  returnData.xtraj = PPTrajectory(pchip(tGridUnique, xGrid(:, uniqueIdx)));

  % returnData.ytraj = ytraj;
  % returnData.controlDataCellArray = controlDataCellArray;

end