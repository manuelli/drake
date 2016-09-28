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
  ytraj = inputStruct.ytraj;


  v = CompassGaitVisualizer(p, inputStruct.xtraj.getOutputFrame);
  playback(v,xtraj,struct('slider',true));

  % reconstruct the control trajectories from the HZD controller
  hzdController = p.controller;
  controlTrajs = hzdController.reconstructControlDataFromTrajectory(xtraj);


  % Plot the Lyapunov function (and it's derivative) for y, which comes from the PD law
  A = [0,1;0,0];
  B = [0;1];

  K = [hzdController.Kp,hzdController.Kd]; % the gain matrix, u = -K x

  temp = A - B*K;
  Q = eye(2);
  S = lyap(temp,Q);

  S_function = @(t) S_traj_function(controlTrajs.traj.y)


  % plot y and ydot
  fig = figure(figCounter);
  clf(fig);
  hold on;
  h = fnplt(controlTrajs.traj.y);
  set(h,'Color','b', 'DisplayName','y')
  h = fnplt(controlTrajs.traj.ydot);

  set(h,'Color','r', 'DisplayName','y dot')
  title('y and ydot');
  legend('show');
  figCounter = figCounter + 1;


  % plot the value function, and it's derivative
  fig = figure(figCounter);
  clf(fig);
  hold on;
  h = fnplt(controlTrajs.traj.S);
  set(h,'Color','b', 'DisplayName','S')
  h = fnplt(controlTrajs.traj.S_dot);

  set(h,'Color','r', 'DisplayName','S dot')
  title('S and Sdot');
  legend('show');
  figCounter = figCounter + 1;


  fig = figure(figCounter);
  clf(fig);
  hold on;
  h = fnplt(controlTrajs.traj.u);
  set(h, 'Color', 'b', 'DisplayName', 'u');

  h = fnplt(controlTrajs.traj.u_fb);
  set(h, 'Color', 'r', 'DisplayName', 'u fb');

  h = fnplt(controlTrajs.traj.uStar);
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
    h = fnplt(xtraj,[3]);
    set(h, 'Color','b', 'DisplayName','swing')

    h = fnplt(xtraj,[4]);
    set(h, 'Color','r')
    title('velocity trajectories', 'DisplayName','stance')
    hold off;

    figCounter = figCounter + 1;
  end


end


% function S_val_matrix = S_traj_function(yTraj, ydotTraj S, t)
%   S_val_matrix = zeros(2,1);

%   S(1) = S*[yTraj.eval(t); ydotTraj.eval(t)]
%   S(2) = S*;
% end