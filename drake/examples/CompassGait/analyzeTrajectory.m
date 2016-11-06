function [controlTrajs] = analyzeTrajectory(inputStruct, options)
  % input struct should have fields
  % plant
  % xtraj
  % ytraj

  if (nargin < 2)
    options = struct();
  end

  defaultOptions.figCounter = 5;
  defaultOptions.allPlots = false;
  options = applyDefaults(options, defaultOptions);


  figCounter = options.figCounter;

  p = inputStruct.plant;
  xtraj = inputStruct.xtraj;
  controlTrajs = inputStruct.controlTrajs;
  % ytraj = inputStruct.ytraj;


  v = CompassGaitVisualizer(p, inputStruct.xtraj.getOutputFrame);
  playback(v,xtraj,struct('slider',true));

  % reconstruct the control trajectories from the HZD controller
  % hzdController = p.controller;
  % controlTrajs = hzdController.reconstructControlDataFromTrajectory(ytraj);


  % plot y and ydot
  fig = figure(figCounter);
  clf(fig);
  hold on;
  h = fnplt(controlTrajs.controlData.y);
  set(h,'Color','b', 'DisplayName','y')
  h = fnplt(controlTrajs.controlData.ydot);

  set(h,'Color','r', 'DisplayName','y dot')
  title('y and ydot');
  legend('show');
  figCounter = figCounter + 1;



  fig = figure(figCounter);
  clf(fig);
  hold on;
  h = fnplt(controlTrajs.controlData.V);
  set(h,'Color','b', 'DisplayName','V')

  title('V');
  legend('show');
  figCounter = figCounter + 1;

  % plot the value function, and it's derivative
  fig = figure(figCounter);
  clf(fig);
  hold on;
  h = fnplt(controlTrajs.controlData.V);
  set(h,'Color','b', 'DisplayName','V')


  h = fnplt(controlTrajs.controlData.V_dot);
  set(h,'Color','r', 'DisplayName','V dot')
  title('S and Sdot');
  legend('show');
  figCounter = figCounter + 1;


  fig = figure(figCounter);
  clf(fig);
  hold on;
  h = fnplt(controlTrajs.controlData.u);
  set(h, 'Color', 'b', 'DisplayName', 'u');

  % h = fnplt(controlTrajs.controlData.u_fb);
  % set(h, 'Color', 'r', 'DisplayName', 'u fb');

  h = fnplt(controlTrajs.controlData.uStar);
  set(h, 'Color', 'g', 'DisplayName', 'u ff');

  title('control input')
  legend('show')
  hold off;

  figCounter = figCounter + 1;



  if (options.allPlots)
    fig = figure(figCounter);
    clf(fig);
    hold on;
    h = fnplt(xtraj,[2]);
    set(h, 'Color','b', 'DisplayName','swing');


    h = fnplt(xtraj,[3]);
    set(h, 'Color','r', 'DisplayName','stance')
    title('position trajectories')
    legend('show')
    hold off;

    figCounter = figCounter + 1;



    fig = figure(figCounter);
    clf(fig);
    hold on;
    h = fnplt(xtraj,[4]);
    set(h, 'Color','b', 'DisplayName','swing');

    h = fnplt(xtraj,[5]);
    set(h, 'Color','r', 'DisplayName', 'stance');
    title('velocity trajectories', 'DisplayName','stance')

    legend('show')
    hold off;

    figCounter = figCounter + 1;
  end


end


% function S_val_matrix = S_traj_function(yTraj, ydotTraj S, t)
%   S_val_matrix = zeros(2,1);

%   S(1) = S*[yTraj.eval(t); ydotTraj.eval(t)]
%   S(2) = S*;
% end