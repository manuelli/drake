% utility method for plotting trajectories coming out of our simulator.
% input data should have fields 
% - trueParticleArray
% - hzdController
% - uArray
% - times

classdef PlotParticleTrajectory < handle
  properties
    inputData;
    plotArrayTemplate_;
    plotArrayEKF_;
    plotArray_;
    plotArrayObserver_;
    figCounter_;
    plotHandles_;
    cgUtils_;
    options_;
  end

  methods
    function obj = PlotParticleTrajectory(inputData, options)
      
      obj.cgUtils_ = CompassGaitUtils();
      obj.inputData = inputData;
      plotArray = struct();
      plotArray.V = [];
      plotArray.V_dot = [];
      plotArray.y = [];
      plotArray.ydot = [];
      plotArray.yddot_nominal = [];
      plotArray.A_y = [];
      plotArray.B_y = [];
      plotArray.qL = [];
      plotArray.qR = [];
      plotArray.vL = [];
      plotArray.vR = [];
      plotArray.uNominal = [];
      
      plotArray.u = [];
      plotArray.phaseVar = [];

      % debugging
      plotArray.A_y_theta = [];
      plotArray.A_y_H = [];
      plotArray.hd_dderiv = [];
      plotArray.phaseVarDot = [];

      %% LQR stuff
      plotArray.uLQR = [];
      plotArray.uLQR_fb = [];
      plotArray.uLQR_plan = [];
      plotArray.V_lqr = []; % the lqr value function

      % PD controller stuff
      plotArray.u_pd = [];

      obj.plotArrayTemplate_ = plotArray;
      obj.plotArray_ = plotArray;
      obj.plotArrayEKF_ = obj.plotArrayTemplate_;
      obj.plotArrayObserver_ = obj.plotArrayTemplate_;



      obj.initializeOptions(options);

      obj = obj.createPlotHandles();
      obj = obj.createPlotArrays();
    end

    function obj = initializeOptions(obj, options)
      defaultOptions = struct();
      defaultOptions.plotEKF = true;
      defaultOptions.plotLQR = false;
      defaultOptions.controlTypeToPlot = 'hzd';

      obj.options_ = applyDefaults(options, defaultOptions);
    end

    function obj = createPlotArrays(obj)
      numTimes = length(obj.inputData.times);

      for i=1:numTimes
        trueParticle = obj.inputData.trueParticleArray{i};
        
        uGlobal = obj.inputData.uArray(i);
        obj.plotArray_ = obj.populatePlotArray(trueParticle, obj.plotArray_, uGlobal, i);

        if obj.options_.plotEKF
          ekfParticle = obj.inputData.kalmanFilterParticleArray{i};
          obj.plotArrayEKF_ = obj.populatePlotArray(ekfParticle, obj.plotArrayEKF_, uGlobal, i);
        end

        if obj.options_.plotObserver
          observerParticle = obj.inputData.observerParticleArray{i};
          obj.plotArrayObserver_ = obj.populatePlotArray(observerParticle, obj.plotArrayObserver_, uGlobal, i);
        end
      end

      
    end

    function plotActualControl(obj)
      fig = figure(obj.plotHandles_.control);
      hold on;
      plot(obj.inputData.times, obj.inputData.uArray, 'm', 'DisplayName', 'actual control');
      legend('show');
      hold off;
    end

    function plot(obj, options)
      if nargin < 2
        options = struct();
      end

      defaultOptions = struct();
      defaultOptions.plotTrue =  true;
      defaultOptions.plotEKF = false;
      options = applyDefaults(options, defaultOptions);

      obj = obj.createPlotHandles();

      if options.plotTrue
        plotOptions = struct();
        obj.plotDataArray(obj.plotArray_);
      end

      if options.plotEKF
        plotOptions = struct();
        plotOptions.lineStyle = '--';
        obj.plotDataArray(obj.plotArrayEKF_, plotOptions);
      end

      if options.plotObserver
        plotOptions = struct();
        plotOptions.lineStyle = '--';
        obj.plotDataArray(obj.plotArrayObserver_, plotOptions);
      end

      obj.plotActualControl()
    end

    function obj = createPlotHandles(obj)
      obj.plotHandles_ = struct();
      figCounter = 35;

      fig = figure(figCounter);
      clf(fig);
      obj.plotHandles_.position = fig;
      figCounter = figCounter + 1;

      fig = figure(figCounter);
      clf(fig);
      obj.plotHandles_.valueFunction_lqr = fig;
      figCounter = figCounter + 1;

      fig = figure(figCounter);
      clf(fig);
      obj.plotHandles_.valueFunction = fig;
      figCounter = figCounter + 1;

      fig = figure(figCounter);
      clf(fig);
      obj.plotHandles_.y = fig;
      figCounter = figCounter + 1;

      fig = figure(figCounter);
      clf(fig);
      obj.plotHandles_.control = fig;
      figCounter = figCounter + 1;

      fig = figure(figCounter);
      clf(fig);
      obj.plotHandles_.hzdOutputDynamics = fig;
      figCounter = figCounter + 1;

      fig = figure(figCounter);
      clf(fig);
      obj.plotHandles_.debug = fig;
      figCounter = figCounter + 1;
    end


    function plotArray = populatePlotArray(obj, particle, plotArray, uGlobal, idx)
      p = particle;
      lyapData = obj.inputData.hzdController.computeLyapunovDataFromGlobalState(p.hybridMode_, p.x_, uGlobal);
      % lyapDataArray{end+1} = lyapData;

      plotArray.V(end+1) = lyapData.V;
      plotArray.V_dot(end+1) = lyapData.V_dot;
      plotArray.y(end+1) = lyapData.y;
      plotArray.ydot(end+1) = lyapData.ydot;
      plotArray.phaseVar(end+1) = lyapData.phaseVar;
      uNominalLocal = obj.inputData.hzdController.uPhaseTraj.eval(lyapData.phaseVar);
      plotArray.uNominal(end+1) = obj.cgUtils_.transformLocalControlToGlobalControl(p.hybridMode_, uNominalLocal);
      [plotArray.u(end+1), controlData] = obj.inputData.hzdController.getControlInputFromGlobalState(0, p.hybridMode_, p.x_);

      plotArray.u_pd(end+1) = obj.inputData.pdController.getControlInputFromGlobalState(p.hybridMode_, p.x_);

      plotArray.A_y(end+1) = lyapData.A_y;
      plotArray.A_y_theta(end+1) = lyapData.A_y_theta;
      plotArray.A_y_H(end+1) = lyapData.A_y_H;
      plotArray.B_y(end+1) = lyapData.B_y;

      % CAREFUL: some of the stuff in control data is in local coords
      plotArray.yddot_nominal(end+1) = controlData.yddot;
      plotArray.qL(end+1) = p.x_.qL;
      plotArray.qR(end+1) = p.x_.qR;
      plotArray.vL(end+1) = p.x_.vL;
      plotArray.vR(end+1) = p.x_.vR;

      % debugging
      plotArray.hd_dderiv(end+1) = obj.inputData.hzdController.hdTraj_dderiv.eval(lyapData.phaseVar);

      if obj.options_.plotLQR
        t_plan = obj.inputData.plan_time_array(idx);
        [uLQR, lqrData] = obj.inputData.cgLQR.getControlInputFromGlobalState(t_plan, p.hybridMode_, p.x_);

        plotArray.uLQR(end+1) = lqrData.u;
        plotArray.uLQR_fb(end+1) = lqrData.u_fb;
        plotArray.uLQR_plan(end+1) = lqrData.u_plan;

        lqrData = obj.inputData.cgLQR.getInfoFromGlobalState(t_plan, p.hybridMode_, p.x_);
        plotArray.V_lqr(end+1) = lqrData.V;
      end

      xLocal = obj.cgUtils_.transformGlobalStateToLocalState(particle.hybridMode_, particle.x_);
      plotArray.phaseVarDot(end+1) = xLocal(4);
    end


    function plotDataArray(obj, plotArray, options)
      if nargin < 3
        options = struct();
      end

      defaultOptions = struct();
      defaultOptions.lineStyle = '-';
      options = applyDefaults(options, defaultOptions);

      tGrid = obj.inputData.times;
      % %% plot Value function
      % figCounter = obj.figCounter_;
      % fig = figure(figCounter);
      % clf(fig);
      fig = figure(obj.plotHandles_.valueFunction);
      subplot(2,1,1);
      hold on;
      plot(tGrid, plotArray.V, strcat(options.lineStyle,'b'), 'DisplayName', 'V');
      title('V hzd');
      legend('show');
      hold off;

      % figCounter = figCounter + 1;
      % fig = figure(figCounter);
      subplot(2,1,2);
      hold on;
      plot(tGrid, plotArray.V_dot, strcat(options.lineStyle,'r'), 'DisplayName', 'V dot');
      legend('show');
      hold off;

      if obj.options_.plotLQR
        fig = figure(obj.plotHandles_.valueFunction_lqr);
        subplot(2,1,1);
        hold on;
        plot(tGrid, plotArray.V_lqr, strcat(options.lineStyle,'b'), 'DisplayName', 'V lqr');
        title('V lqr');
        legend('show');
        hold off;
      end
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% plot y values
      % figCounter = figCounter + 1;
      % fig = figure(figCounter);
      % clf(fig);
      figure(obj.plotHandles_.y);
      numPlots = 3;
      subplot(numPlots,1,1);
      hold on;
      plot(tGrid, plotArray.y, strcat(options.lineStyle,'b'), 'DisplayName', 'y');
      title('y');
      legend('show');
      hold off;

      % figCounter = figCounter + 1;
      % fig = figure(figCounter);
      subplot(numPlots,1,2);
      hold on;
      plot(tGrid, plotArray.ydot, strcat(options.lineStyle,'b'), 'DisplayName', 'y dot')
      legend('show');
      hold off;

      subplot(numPlots,1,3);
      hold on;
      plot(tGrid, plotArray.yddot_nominal, strcat(options.lineStyle,'b'), 'DisplayName', 'yddot normal control')
      legend('show');
      hold off;

      figure(obj.plotHandles_.hzdOutputDynamics);
      numPlots = 2;
      subplot(numPlots,1,1);
      hold on;
      plot(tGrid, plotArray.A_y, strcat(options.lineStyle,'b'), 'DisplayName', 'Ay');
      title('y');
      legend('show');
      hold off;

      % figCounter = figCounter + 1;
      % fig = figure(figCounter);
      subplot(numPlots,1,2);
      hold on;
      plot(tGrid, plotArray.B_y, strcat(options.lineStyle,'b'), 'DisplayName', 'By')
      legend('show');
      hold off;



      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% plot position/velocity
      % figCounter = figCounter + 1;
      % fig = figure(figCounter);
      % clf(fig);
      figure(obj.plotHandles_.position)
      subplot(2,1,1);
      hold on;
      plot(tGrid, plotArray.qL, strcat(options.lineStyle,'r'), 'DisplayName', 'qL');
      plot(tGrid, plotArray.qR, strcat(options.lineStyle,'b'), 'DisplayName', 'qR');
      title('position');
      legend('show');
      hold off;

      subplot(2,1,2);
      hold on;
      plot(tGrid, plotArray.vL, strcat(options.lineStyle,'r'), 'DisplayName', 'vL');
      plot(tGrid, plotArray.vR, strcat(options.lineStyle,'b'), 'DisplayName', 'vR');
      title('velocity');
      hold off;

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% plot control input
      %% plot position/velocity
      % figCounter = figCounter + 1;
      % fig = figure(figCounter);
      % clf(fig);

      figure(obj.plotHandles_.control);
      % subplot(2,1,1);
      hold on;
      switch obj.options_.controlTypeToPlot
        case 'hzd'
          plot(tGrid, plotArray.u, strcat(options.lineStyle,'b'), 'DisplayName', 'u hzd');
        case 'pd'
          plot(tGrid, plotArray.u_pd, strcat(options.lineStyle,'b'), 'DisplayName', 'u pd');
        otherwise
          error('supported control types are pd and hzd'); 
      end
          
      plot(tGrid, plotArray.uNominal, strcat(options.lineStyle,'g'), 'DisplayName', 'u plan');

      if obj.options_.plotLQR
        plot(tGrid, plotArray.uLQR, strcat(options.lineStyle,'r'), 'DisplayName', 'uLQR');
      end
      title('control input');
      legend('show');
      hold off;

      figure(obj.plotHandles_.debug);
      numPlots = 3;
      subplot(numPlots,1,1);
      hold on;
      plot(tGrid, plotArray.A_y_theta, strcat(options.lineStyle,'b'), 'DisplayName', 'Ay theta');
      plot(tGrid, plotArray.A_y_H, strcat(options.lineStyle,'r'), 'DisplayName', 'Ay H');
      title('control input');
      legend('show');
      hold off;

      subplot(numPlots,1,2)
      hold on;
      plot(tGrid, plotArray.hd_dderiv, strcat(options.lineStyle,'b'), 'DisplayName', 'hd dderiv');
      legend('show');
      hold off;

      subplot(numPlots,1,3)
      hold on;
      plot(tGrid, plotArray.phaseVarDot.^2, strcat(options.lineStyle,'b'), 'DisplayName', 'theta dot squared');
      legend('show');
      hold off;
    end

  end

end
% function plotParticleTrajectory(inputData)
%   V = [];
%   V_dot = [];
%   y = [];
%   ydot = [];
%   qL = [];
%   qR = [];
%   vL = [];
%   vR = [];
%   uNominal = [];
%   uTrueParticle = [];
%   lyapDataArray = {};
%   numParticles = length(inputData.times);
%   tGrid = inputData.times;

%   cgUtils = CompassGaitUtils();

%   plotArray = struct();
%   plotArray.V = [];
%   plotArray.V_dot = [];
%   plotArray.y = [];
%   plotArray.ydot = [];
%   plotArray.qL = [];
%   plotArray.qR = [];
%   plotArray.vL = [];
%   plotArray.vR = [];
%   plotArray.uNominal = [];

%   % just create copy for ekf stuff
%   plotArrayEKF = plotArray; 




%   V_ekf = [];
%   V_dot_ekf = [];
%   y_ekf = [];
%   ydot_ekf = [];
%   qL_ekf = [];
%   qR_ekf = [];
%   vL_ekf = [];
%   vR_ekf = [];
%   uNominal_ekf = [];
%   lyapDataArray_ekf = {};
%   ekfActive = numel(inputData.kalmanFilterParticleArray);
%   ekfActive = false;

%   for i=1:numParticles
%     p = inputData.trueParticleArray{i};
%     lyapData = inputData.hzdController.computeLyapunovDataFromGlobalState(p.hybridMode_, p.x_, inputData.uArray(i));
%     lyapDataArray{end+1} = lyapData;

%     V(end+1) = lyapData.V;
%     V_dot(end+1) = lyapData.V_dot;
%     y(end+1) = lyapData.y;
%     ydot(end+1) = lyapData.ydot;
%     phaseVar = lyapData.phaseVar;
%     uNominalLocal = inputData.hzdController.uPhaseTraj.eval(phaseVar);
%     uNominal(end+1) = cgUtils.transformLocalControlToGlobalControl(p.hybridMode_, uNominalLocal);
%     uTrueParticle(end+1) = inputData.hzdController.getControlInputFromGlobalState(tGrid(i), p.hybridMode_, p.x_);
%     qL(end+1) = p.x_.qL;
%     qR(end+1) = p.x_.qR;
%     vL(end+1) = p.x_.vL;
%     vR(end+1) = p.x_.vR;

%     if ekfActive
%       p = inputData.kalmanFilterParticleArray{i};
%       lyapData = inputData.hzdController.computeLyapunovDataFromGlobalState(p.hybridMode_, p.x_, inputData.uArray(i));
%       lyapDataArray_ekf{end+1} = lyapData;

%       V_ekf(end+1) = lyapData.V;
%       V_dot_ekf(end+1) = lyapData.V_dot;
%       y_ekf(end+1) = lyapData.y;
%       ydot_ekf(end+1) = lyapData.ydot;
%       phaseVar_ekf = lyapData.phaseVar;
%       uNominalLocal = inputData.hzdController.uPhaseTraj.eval(phaseVar);
%       uNominal_ekf(end+1) = cgUtils.transformLocalControlToGlobalControl(p.hybridMode_, uNominalLocal);
%       qL_ekf(end+1) = p.x_.qL;
%       qR_ekf(end+1) = p.x_.qR;
%       vL_ekf(end+1) = p.x_.vL;
%       vR_ekf(end+1) = p.x_.vR;
%     end
%   end

%   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   %% plot Value function
%   figCounter = 35;
%   fig = figure(figCounter);
%   clf(fig);
%   subplot(2,1,1);
%   hold on;
%   plot(tGrid, V, 'b', 'DisplayName', 'V');
%   title('V');
%   legend('show');
%   hold off;

%   % figCounter = figCounter + 1;
%   % fig = figure(figCounter);
%   subplot(2,1,2);
%   hold on;
%   plot(tGrid, V_dot, 'r', 'DisplayName', 'V dot');
%   legend('show');
%   hold off;

%   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   %% plot y values
%   figCounter = figCounter + 1;
%   fig = figure(figCounter);
%   clf(fig);
%   subplot(2,1,1);
%   hold on;
%   plot(tGrid, y, 'b', 'DisplayName', 'y');
%   title('y');
%   legend('show');
%   hold off;

%   % figCounter = figCounter + 1;
%   % fig = figure(figCounter);
%   subplot(2,1,2);
%   hold on;
%   plot(tGrid, ydot, 'r', 'DisplayName', 'y dot')
%   legend('show');
%   hold off;



%   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   %% plot position/velocity
%   figCounter = figCounter + 1;
%   fig = figure(figCounter);
%   clf(fig);
%   subplot(2,1,1);
%   hold on;
%   plot(tGrid, qL, 'r', 'DisplayName', 'qL');
%   plot(tGrid, qR, 'b', 'DisplayName', 'qR');
%   title('position');
%   legend('show');
%   hold off;

%   subplot(2,1,2);
%   hold on;
%   plot(tGrid, vL, 'r', 'DisplayName', 'vL');
%   plot(tGrid, vR, 'b', 'DisplayName', 'vR');
%   title('velocity');
%   hold off;

%   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   %% plot control input
%   %% plot position/velocity
%   figCounter = figCounter + 1;
%   fig = figure(figCounter);
%   clf(fig);
%   % subplot(2,1,1);
%   hold on;
%   plot(tGrid, inputData.uArray, 'b', 'DisplayName', 'u actual');
%   plot(tGrid, uTrueParticle, 'r', 'DisplayName', 'u true particle');
%   plot(tGrid, uNominal, 'g', 'DisplayName', 'u plan');

%   title('control input');
%   legend('show');
%   hold off;

% end