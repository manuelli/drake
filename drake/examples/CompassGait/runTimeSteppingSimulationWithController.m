function returnData = runTimeSteppingSimulationWithController(plant, controller, tspan, dt, x_initial)

  tGrid = [tspan(1):dt:tspan(2)];

  numTimes = length(tGrid);

  xtrajCellArray = {};
  ytrajCellArray = {};
  controlDataCellArray = {};
  uGrid = [];
  hybridEventTimes = [];

  returnData = struct();


  x_current = x_initial;
  for i=1:(numTimes-1)
    t = tGrid(i);
    [u, controlData] = controller.tick(t,x_current);
    t_start = tGrid(i);
    t_end = tGrid(i+1);
    [ytrajShort, xtrajShort] = plant.simulateWithConstantControlInput(x_current, [t_start, t_end], u);

    x_current = xtrajShort.eval(t_end); % update x_current for the next simulation cycle

    % there is a bit of special casing for the first tick
    if(i == 1)
      uGrid = zeros(length(u), numTimes-1);
      xtraj = xtrajShort;
      ytraj = ytrajShort;
      continue;
    end

    uGrid(:, i) = u;
    controlDataCellArray{end+1} = controlData;

    % append the trajectories
    % need to be careful to deal with hybrid trajectories here


    if isa(xtrajShort, 'HybridTrajectory')
      numTrajs = length(xtrajShort.traj);
      for j=1:numTrajs
        xtraj = xtraj.append(xtrajShort.traj{j});
        ytraj = ytraj.append(ytrajShort.traj{j});
      end

      hybridEventTimes = [hybridEventTimes, xtrajShort.te];
    else
      % standard case, can just do appends
      xtraj = xtraj.append(xtrajShort);
      ytraj = ytraj.append(ytrajShort);
    end
    
  end

  returnData.utraj = PPTrajectory(pchip(tGrid(1:numTimes-1), uGrid));
  returnData.hybridEventTimes = hybridEventTimes;
  returnData.xtraj = xtraj;
  returnData.ytraj = ytraj;
  returnData.controlDataCellArray = controlDataCellArray;

end