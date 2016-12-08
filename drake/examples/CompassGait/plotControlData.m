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

  u_mode_1 = 0*uActual_grid;
  u_mode_2 = 0*uActual_grid;

  num_mode_1_particles = 0*uActual_grid;
  num_mode_2_particles = 0*uActual_grid;

  for i=1:length(idxRange)
    idx = idxRange(i);
    d = CompassGaitParticle.getAvgParticleInEachMode(inputData.particleSetArray{idx});
    num_mode_1_particles(i) = numel(d.mode1);
    num_mode_2_particles(i) = numel(d.mode2);

    if (num_mode_1_particles > 0)
      u_mode_1(i) = inputData.hzdController.getControlInputFromGlobalState(t_grid(i), d.mode1_avg.hybridMode_, d.mode1_avg.x_);
    end

    if (num_mode_2_particles > 0)
      u_mode_2(i) = inputData.hzdController.getControlInputFromGlobalState(t_grid(i), d.mode2_avg.hybridMode_, d.mode2_avg.x_);
    end
  end


  numPlots = 2;

  fig = figure(figCounter);
  
  subplot(numPlots,1,1)
  hold on;
  plot(t_grid, uActual_grid, 'g', 'DisplayName', 'u actual');
  plot(t_grid, u_mode_1, 'r', 'DisplayName', 'u mode 1 ');
  plot(t_grid, u_mode_2, 'b', 'DisplayName', 'u mode 2 ');
  legend('show');
  ylabel('control input')
  xlabel('time')
  title('control input in each mode')
  hold off;

  subplot(numPlots,1,2);
  hold on;
  particleFractions = num_mode_1_particles*1.0/(num_mode_1_particles + num_mode_2_particles);
  plot(t_grid, particleFractions)

  ylabel('fraction of particles in mode 1');
  xlabel('time');
  title('fractions of particles in mode 1');
  hold off;

end