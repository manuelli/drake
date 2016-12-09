classdef PlotUtility < handle

  properties
    inputData_;
    idx_;
    uGlobal_;
    figHandle_;
    numPlots_;
    cgUtils_;

    timeSlider_;
    controlSlider_;
    resetControlButton_;
  end

  methods
    function obj = PlotUtility(inputData)
      obj.inputData_ = inputData;
      if(~isfield(inputData,'figHandle_'))
        obj.figHandle_ = figure(30);
      else
        obj.figHandle_ = inputData.figHandle_;
      end
      obj.numPlots_ = 3;
      obj.cgUtils_ = CompassGaitUtils();
    end

    function setIdx(obj, idx)
      obj.idx_ = idx;
    end

    function setControl(obj, uGlobal)
      obj.uGlobal_ = uGlobal;
    end

    function updatePlots(obj, particleSet, uGlobal)
      fig = figure(obj.figHandle_);
      subplot(obj.numPlots_,1,1);
      cla reset;

      % particleSet = obj.inputData_.particleSetArray(obj.idx_);
      d = CompassGaitParticle.getAvgParticleInEachMode(particleSet);
      modeSets = {};
      modeSets{1} = d.mode1;
      modeSets{2} = d.mode2;

      num_mode = zeros(2,1);
      mode_bar_plot_idx = {[],[]};
      vdot_mode = {[],[]};
      v_mode = {[],[]};

      for modeIdx=1:2
        num_mode(modeIdx) = numel(modeSets{modeIdx});
        for i = 1:num_mode(modeIdx)
          particle = modeSets{modeIdx}{i};
          data = obj.getInfoForParticle(particle, uGlobal);
          vdot_mode{modeIdx}(end+1) = data.V_dot;
          v_mode{modeIdx}(end+1) = data.V;
        end
      end

      mode_bar_plot_idx{1} = 1:num_mode(1);
      mode_bar_plot_idx{2} = (num_mode(1) + 1):(num_mode(1) + num_mode(2));

      % make bar plot
      hold on;
      if (num_mode(1) > 0)
        bar(mode_bar_plot_idx{1}, vdot_mode{1}, 'r');
      end
      if (num_mode(2) > 0)
        bar(mode_bar_plot_idx{2}, vdot_mode{2}, 'b')
      end
      hold off;
      ylabel('V dot')

      subplot(obj.numPlots_,1,2)
      cla reset;
      hold on;
      if (num_mode(1) > 0)
        bar(mode_bar_plot_idx{1}, v_mode{1}, 'r');
      end
      if (num_mode(2) > 0)
        bar(mode_bar_plot_idx{2}, v_mode{2}, 'b')
      end
      hold off;
      title('V')

      obj.plotRobustCostFunction(particleSet);
    end

    function plotRobustCostFunction(obj, particleSet)
      dt = 0.005; % just hacked for now
      [u_opt, data] = obj.inputData_.hzdController.computeRobustControlFromParticleSet(particleSet, dt);

      ustep = 0.01;
      uRange = -5:ustep:5;
      costVal = 0*uRange;

      for i=1:length(uRange)
        u = uRange(i);
        costVal(i) = data.costFun(u);
      end

      idx = floor(get(obj.timeSlider_,'Value'));
      uGlobalTraj = obj.inputData_.uArray(idx);

      uGlobalSlider = get(obj.controlSlider_,'Value');

      subplot(obj.numPlots_,1,3);
      cla reset;
      hold on;
      plot(uRange, costVal, 'b')
      scatter(uGlobalTraj, data.costFun(uGlobalTraj), 'filled', 'g');
      scatter(uGlobalSlider, data.costFun(uGlobalSlider), 'filled', 'r');
      ylabel('robust cost');
      xlabel('control input');
      title('Robust Cost Function');
      hold off;
    end

    function returnData = getInfoForParticle(obj, particle, uGlobal)
      returnData = obj.inputData_.hzdController.computeLyapunovDataFromGlobalState(particle.hybridMode_, particle.x_, uGlobal);
    end

    function timeCallback(obj, source, event)
      idx = floor(get(source,'Value'));
      particleSet
    end

    function controlCallback(obj, source, event)
      obj.updatePlotsFromSliders();
    end

    function updatePlotsFromSliders(obj, source, event)
      if nargin < 3;
        source = 0;
        event = 0;
      end

      idx = floor(get(obj.timeSlider_,'Value'));
      uGlobal = get(obj.controlSlider_,'Value');

      t = obj.inputData_.times(idx);
      particleSet = obj.inputData_.particleSetArray{idx};

      obj.updatePlots(particleSet,uGlobal);
      titleString = strcat('t = ', num2str(t), ' u =    ', num2str(uGlobal));
      title(titleString);
    end

    function resetControl(obj, source, event)
      if nargin < 3;
        source = 0;
        event = 0;
      end
      idx = floor(get(obj.timeSlider_,'Value'));
      uGlobal = obj.inputData_.uArray(idx);
      set(obj.controlSlider_,'Value',uGlobal);
      obj.updatePlotsFromSliders(0,0);
    end


    function setupPlots(obj)
      fig = figure(obj.figHandle_);
      clf(fig);
      hold on;
      minRange = obj.inputData_.idxRange(1);
      maxRange = obj.inputData_.idxRange(end);
      numIdx = max(maxRange - minRange,1);

      minorStep = 1.0/numIdx;

      callback = @(source, event) obj.updatePlotsFromSliders(source, event);

      obj.timeSlider_ = uicontrol('Style', 'slider', 'Min', minRange, 'Max', maxRange, ...
    'Value', minRange, 'Callback', callback, 'Position', [400 20 120 20], 'SliderStep', [minorStep, 0.1], 'String', 'time');



      controlMin = -5;
      controlMax = 5;
      stepSize = 0.02;
      minorStep = stepSize/(controlMax - controlMin);
      majorStep = 0.1;


      obj.controlSlider_ = uicontrol('Style', 'slider', 'Min', controlMin, 'Max', controlMax, ...
    'Value', 0, 'Callback', callback, 'Position', [20 20 120 20], 'SliderStep', [minorStep, 0.1], 'String', 'control input');

      resetControlCallback = @(source, event) obj.resetControl(source, event);

      % maybe add button for resetting to the planned control input in uArray
      obj.resetControlButton_ = uicontrol('Style', 'pushbutton', 'String', 'Reset Control',...
        'Position', [200 20 50 20],...
        'Callback', resetControlCallback); 
      
      obj.resetControl();
    end
  end
end