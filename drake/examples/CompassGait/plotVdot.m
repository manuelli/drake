function plotVdot(hzdController, controlTrajs, t_center, xtraj)
  figCounter = 20;
  fig = figure(figCounter);
  figCounter = figCounter + 1;
  clf(fig);
  set(fig,'Visible', 'off');

  tBreaks = controlTrajs.controlData.y.getBreaks();


  intervalTime = 0.1;
  tMin = t_center - intervalTime;
  tMax = t_center + intervalTime;


  sld = uicontrol('Style', 'slider', 'Min', t_center - 0.2, 'Max', t_center + 0.2, ...
    'Value', t_center, 'Callback', @showPlot, 'Position', [400 20 120 20]);


%   showPlot(struct('Value',t_center), 0);
  set(fig,'Visible', 'on');


  % handles for keeping track of what we have plotted
  handle = {};


  tGrid = linspace(tMin, tMax, 40);
  VGrid = controlTrajs.controlData.V.eval(tGrid);
  VTildeGrid = controlTrajs.controlDataOther.V.eval(tGrid);

  uActualGrid = 0*tGrid;
  uRobustGrid = 0*tGrid;


  % figure out what the robust control input would have been
  for i=1:length(tGrid)
    t = tGrid(i);
    x = xtraj.eval(t);
    % now figure out what the uncertainty aware controller would have done
    [uRobust, robustData] = hzdController.computeUncertaintyAwareControlInput(x);
    uActual = controlTrajs.controlData.u.eval(t);

    uActualGrid(i) = uActual;
    uRobustGrid(i) = uRobust;
  end

  subplot(3,1,2);
  hold on;
  plot(tGrid, VGrid, 'b', 'DisplayName', 'V');
  plot(tGrid, VTildeGrid, 'r', 'DisplayName', 'V tilde');
  title('V and V tilde')
  hold off;

  subplot(3,1,3);
  hold on;
  title('u and uRobust')
  plot(tGrid, uActualGrid, 'b');
  plot(tGrid, uRobustGrid, 'r');
  hold off;


  function showPlot(source, event)
    t = get(source,'Value');
    % t = getClosestTime(t);
    S = hzdController.S;

    x = xtraj.eval(t);

    y = controlTrajs.controlData.y.eval(t);
    ydot = controlTrajs.controlData.ydot.eval(t);
    A_y = controlTrajs.controlData.A_y.eval(t);
    B_y = controlTrajs.controlData.B_y.eval(t);

    V_actual = controlTrajs.controlData.V.eval(t);
    V_dot_actual = controlTrajs.controlData.V_dot.eval(t);

    yTilde = controlTrajs.controlDataOther.y.eval(t);
    ydotTilde = controlTrajs.controlDataOther.ydot.eval(t);
    A_y_Tilde = controlTrajs.controlDataOther.A_y.eval(t);
    B_y_Tilde = controlTrajs.controlDataOther.B_y.eval(t);

    V_Tilde_actual = controlTrajs.controlDataOther.V.eval(t);
    V_dot_Tilde_actual = controlTrajs.controlDataOther.V_dot.eval(t);

    uActual = controlTrajs.controlData.u.eval(t);


    lb = min(-3,uActual);
    ub = max(3,uActual);
    uGrid = linspace(lb,ub);
    onesGrid = ones(size(uGrid));


    V_dot = 2*[y, ydot] * S * [ydot*onesGrid; A_y + B_y*uGrid];

    % V_dot_test = 2*[y, ydot] * S * [ydot; A_y + B_y*uActual]
    % V_dot_actual

    % need a minus here because the control input is synced with the mode and
    % switches sign during mode changes

    V_dot_Tilde = 2*[yTilde, ydotTilde] * S * [ydotTilde*onesGrid; A_y_Tilde - B_y_Tilde*uGrid];


    % now figure out what the uncertainty aware controller would have done
    [uRobust, robustData] = hzdController.computeUncertaintyAwareControlInput(x);


%     clf(fig);

    for i=1:length(handle)
        delete(handle{i});
    end

    subplot(3,1,1)
    hold on;
    handle = {};  
    handle{end+1} = plot(uGrid, V_dot, 'b', 'DisplayName', 'V dot');
    handle{end+1} = plot(uGrid, V_dot_Tilde, 'r', 'DisplayName', 'V dot Tilde');
    handle{end+1} = scatter(uActual, V_dot_actual, 150, 'g', 'filled', 'DisplayName', 'V dot actual');
    handle{end+1} = scatter(uActual, V_dot_Tilde_actual, 150, 'm', 'filled', 'DisplayName', 'V dot tilde actual');

    handle{end+1} = scatter(uRobust, robustData.V_dot, 150, 'c', 'filled', 'DisplayName', 'V dot robust');

    handle{end+1} = scatter(uRobust, robustData.V_dot_other, 150, 'y', 'filled', 'DisplayName', 'V dot robust tilde');

    % legend('show');
    titleString = strcat('t = ', num2str(t));
    title(titleString);
    hold off
 
    % % plot the V values
    subplot(3,1,2)
    hold on;
    handle{end+1} = scatter(t,V_actual, 150, 'b', 'filled');
    handle{end+1} = scatter(t, V_Tilde_actual, 150, 'r', 'filled');
    hold off;

    % % plot the u values
    subplot(3,1,3)
    hold on;
    handle{end+1} = scatter(t,uActual, 150, 'b', 'filled');
    handle{end+1} = scatter(t, uRobust, 150, 'r', 'filled');
    hold off;

    
  end


  function t_closest = getClosestTime(t)
    [~, idx] = min(abs(t - tBreaks));
    t_closest = tBreaks(idx);
  end

end