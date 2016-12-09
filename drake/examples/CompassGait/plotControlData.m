% utility function for plotting control data stuff around uncertainty boundaries
% inputData should have 
% - ekf
% - particleFilter
% - idxRange over which we should plot
% - tArray
% - controlInputArray
% - hzdController

function plotControlData(inputData)
  figCounter = 10;

  idxRange = inputData.idxRange;

  t_grid = inputData.times(idxRange);
  uActual_grid = inputData.uArray(idxRange);

  particleFilter = inputData.particleFilter;

  u_mode_1 = 0*uActual_grid;
  u_mode_2 = 0*uActual_grid;
  u_robust= 0*uActual_grid;

  num_mode_1_particles = 0*uActual_grid;
  num_mode_2_particles = 0*uActual_grid;
  dataArray = {};

  dt_hack = 0.005; % this doesn't actually affect anything for now

  for i=1:length(idxRange)
    idx = idxRange(i);
    d = CompassGaitParticle.getAvgParticleInEachMode(inputData.particleSetArray{idx});
    dataArray{end+1} = d;
    num_mode_1_particles(i) = numel(d.mode1);
    num_mode_2_particles(i) = numel(d.mode2);

    if (num_mode_1_particles(i) > 0)
      u_mode_1(i) = inputData.hzdController.getControlInputFromGlobalState(t_grid(i), d.mode1_avg.hybridMode_, d.mode1_avg.x_);
    end

    if (num_mode_2_particles(i) > 0)
      u_mode_2(i) = inputData.hzdController.getControlInputFromGlobalState(t_grid(i), d.mode2_avg.hybridMode_, d.mode2_avg.x_);
    end

    particleSet = inputData.particleSetArray{idx};
    if (numel(particleSet) > 0)
      u_robust(i) = inputData.hzdController.computeRobustControlFromParticleSet(particleSet, dt_hack);
    end


  end


  numPlots = 2;

  fig = figure(figCounter);
  clf(fig);
  subplot(numPlots,1,1)
  hold on;
  plot(t_grid, uActual_grid, 'g', 'DisplayName', 'u actual');
  plot(t_grid, u_mode_1, 'r', 'DisplayName', 'u mode 1 ');
  plot(t_grid, u_mode_2, 'b', 'DisplayName', 'u mode 2 ');
  plot(t_grid, u_robust, '--m', 'DisplayName', 'u robust ');
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