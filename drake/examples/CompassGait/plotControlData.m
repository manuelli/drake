% utility function for plotting control data stuff around uncertainty boundaries
% inputData should have 
% - ekf
% - particleFilter
% - idxRange over which we should plot
% - tArray
% - controlInputArray
% - hzdController

function plotControlData(inputData, inputOptions)
  figCounter = 10;

  idxRange = inputData.idxRange;

  t_grid = inputData.times(idxRange);
  uActual_grid = inputData.uArray(idxRange);

  particleFilter = inputData.particleFilter;

  u_true_state = 0*uActual_grid;
  u_mode_1 = 0*uActual_grid;
  u_mode_1_robust = 0*uActual_grid;
  u_mode_2_robust = 0*uActual_grid;
  u_mode_2 = 0*uActual_grid;
  u_robust = 0*uActual_grid;
  u_robust_blend = 0*uActual_grid;
  u_blend = 0*uActual_grid;

  u_particle_standard_control = 0*uActual_grid;

  u_robust_pd = 0*uActual_grid;

  u_observers = {}; % record the control inputs we would have gotten from the observers
  num_observers = 0;
  if isfield(inputData, 'observerBankParticleArray')
    num_observers = length(inputData.observerArray);
    for i=1:num_observers
      u_observers{end+1} = 0*uActual_grid;
    end
  end

  num_mode_1_particles = 0*uActual_grid;
  num_mode_2_particles = 0*uActual_grid;
  mode_1_fraction = 0*uActual_grid;
  dataArray = {};

  dt_hack = 0.005; % this doesn't actually affect anything for now

  for i=1:length(idxRange)
    idx = idxRange(i);
    d = CompassGaitParticle.getAvgParticleInEachMode(inputData.particleSetArray{idx});
    dataArray{end+1} = d;
    num_mode_1_particles(i) = numel(d.mode1);
    num_mode_2_particles(i) = numel(d.mode2);
    mode_1_fraction(i) = num_mode_1_particles(i)*1.0/(max(1,num_mode_1_particles(i)+ num_mode_2_particles(i)));

    if (num_mode_1_particles(i) > 0)
      u_mode_1(i) = inputData.hzdController.getControlInputFromGlobalState(t_grid(i), d.mode1_avg.hybridMode_, d.mode1_avg.x_);

      u_mode_1_robust(i) = inputData.hzdController.computeRobustControlFromParticleSet(d.mode1, dt_hack);
    end

    if (num_mode_2_particles(i) > 0)
      u_mode_2(i) = inputData.hzdController.getControlInputFromGlobalState(t_grid(i), d.mode2_avg.hybridMode_, d.mode2_avg.x_);

      u_mode_2_robust(i) = inputData.hzdController.computeRobustControlFromParticleSet(d.mode2, dt_hack);
    end

    particleSet = inputData.particleSetArray{idx};
    if (numel(particleSet) > 0)
      u_robust(i) = inputData.hzdController.computeRobustControlFromParticleSet(particleSet, dt_hack);

      u_robust_pd(i) = inputData.pdController.computeRobustControlFromParticleSet(particleSet);
    end

    u_blend(i) = mode_1_fraction(i)*u_mode_1(i) + (1-mode_1_fraction(i))*u_mode_2(i);
    u_robust_blend(i) = mode_1_fraction(i)*u_mode_1_robust(i) + (1-mode_1_fraction(i))*u_mode_2_robust(i);

    if isfield(inputData, 'observerBankParticleArray')
      num_observers = length(inputData.observerArray);
      for observerIdx=1:num_observers
        observerParticle = inputData.observerBankParticleArray{observerIdx}{idx};
        u_observers{observerIdx}(i) = inputData.hzdController.getControlInputFromGlobalState(t_grid(i), observerParticle.hybridMode_, observerParticle.x_);

        if strcmp(inputOptions.controlTypeToPlot, 'pd')
          u_observers{observerIdx}(i) = inputData.pdController.getControlInputFromGlobalState(observerParticle.hybridMode_, observerParticle.x_);
        end

      end
    end

    % compute standard (i.e. non-robust control input using most likely avg from particle set)
    if (numel(particleSet) > 0)
      particleSetAvgData = CompassGaitParticle.getAvgParticleInMostLikelyMode(particleSet);

      particle = particleSetAvgData.avgParticleInMostLikelyMode;


      switch inputOptions.controlTypeToPlot
        case 'hzd'
         u_particle_standard_control(i) = inputData.hzdController.getControlInputFromGlobalState(t_grid(i), particle.hybridMode_, particle.x_);
        case 'pd'
        u_particle_standard_control(i) = inputData.pdController.getControlInputFromGlobalState(particle.hybridMode_, particle.x_); 
      end
    end

    trueParticle = inputData.trueParticleArray{idx};

    
    switch inputOptions.controlTypeToPlot
      case 'pd'
        u_true_state(i) = inputData.pdController.getControlInputFromGlobalState(trueParticle.hybridMode_, trueParticle.x_);
      case 'hzd'
        u_true_state(i) = inputData.hzdController.getControlInputFromGlobalState(t_grid(i), trueParticle.hybridMode_, trueParticle.x_);
    end

  end


  numPlots = 2;

  fig = figure(figCounter);
  clf(fig);
  subplot(numPlots,1,1)
  hold on;

  plot(t_grid, u_true_state, 'c', 'DisplayName', 'u true state');
  plot(t_grid, uActual_grid, 'g', 'DisplayName', 'u actual');

  

  % extra plotting stuff that I don't want right now
  if false
    idx = abs(u_mode_1) > 0;
    plot(t_grid(idx), u_mode_1(idx), 'r', 'DisplayName', 'u mode 1 ');

    idx = abs(u_mode_2) > 0;
    plot(t_grid(idx), u_mode_2(idx), 'b', 'DisplayName', 'u mode 2 ');

    idx = abs(u_mode_1_robust) > 0;
    plot(t_grid(idx), u_mode_1_robust(idx), '--r', 'DisplayName', 'u mode 1 robust');

    idx = abs(u_mode_2_robust) > 0;
    plot(t_grid(idx), u_mode_2_robust(idx), '--b', 'DisplayName', 'u mode 2 robust');
  end
  
  % the robust control input
  idx = abs(u_robust) > 0;

  switch inputOptions.controlTypeToPlot
    case 'pd'
      plot(t_grid(idx), u_robust_pd(idx), 'r', 'Displayname', 'u robust pd');
    case 'hzd'
      plot(t_grid(idx), u_robust(idx), 'm', 'DisplayName', 'u robust ');
  end
  

  idx = abs(u_particle_standard_control) > 0;
  plot(t_grid(idx), u_particle_standard_control(idx), '--m', 'DisplayName', 'u particle standard');

  if (num_observers > 0)
    colorString = ['g','r','b'];
    for i=1:num_observers
      plotString = strcat('--', colorString(i));
      name = inputData.observerArray{i}.name_;
      plot(t_grid, u_observers{i}, plotString, 'DisplayName', name);
    end
  end
  % plot(t_grid, u_robust_blend, '--m', 'DisplayName', 'u robust blend');
  % plot(t_grid, u_blend, 'c', 'DisplayName', 'u blend ');
  legend('show');
  ylabel('control input')
  xlabel('time')
  title('control input in each mode')
  hold off;

  subplot(numPlots,1,2);
  hold on;
  particleFractions = num_mode_1_particles*1.0./(num_mode_1_particles + num_mode_2_particles);
  plot(t_grid, particleFractions)

  ylabel('fraction of particles in mode 1');
  xlabel('time');
  title('fractions of particles in mode 1');
  hold off;



  figCounter = figCounter + 1;
  fig = figure(figCounter);
  clf(fig);
  
  subplot(2,1,1)
  hold on;
  defaultPlotOptions = struct();
  plotTypeString = 'position';
  defaultPlotOptions.plotType = plotTypeString;
  defaultPlotOptions.nominalPlotType = 'position';
  particleFilter.plotNominalTraj(defaultPlotOptions)

  for i=1:numel(idxRange)
    % plot the true particle and avg particles in each mode
    trueParticleIdx = idxRange(i);
    particleFilter.plotSingleParticle(inputData.trueParticleArray{trueParticleIdx}, struct('colorString','g', 'plotType', plotTypeString));


    % plot particle in each mode (if there is one)
    if (num_mode_1_particles(i) > 0)
      particleFilter.plotSingleParticle(dataArray{i}.mode1_avg, defaultPlotOptions);
    end

    if (num_mode_2_particles(i) > 0)
      particleFilter.plotSingleParticle(dataArray{i}.mode2_avg, defaultPlotOptions);
    end


  end
  hold off;

  subplot(2,1,2)
  hold on;
  defaultPlotOptions = struct();
  plotTypeString = 'left';
  defaultPlotOptions.plotType = plotTypeString;
  defaultPlotOptions.nominalPlotType = 'normal';
  particleFilter.plotNominalTraj(defaultPlotOptions)

  for i=1:numel(idxRange)
    % plot the true particle and avg particles in each mode
    trueParticleIdx = idxRange(i);
    particleFilter.plotSingleParticle(inputData.trueParticleArray{trueParticleIdx}, struct('colorString','g', 'plotType', plotTypeString));


    % plot particle in each mode (if there is one)
    if (num_mode_1_particles(i) > 0)
      particleFilter.plotSingleParticle(dataArray{i}.mode1_avg, defaultPlotOptions);
    end

    if (num_mode_2_particles(i) > 0)
      particleFilter.plotSingleParticle(dataArray{i}.mode2_avg, defaultPlotOptions);
    end


  end
  hold off;

end