% interactive window for plotting the particles
% try to plot whatever was passed in
% inputData should have fields
% - particleFilter
% - particleSetArray
% - times
% - trueParticleArray
% 
function plotParticles(inputData, figHandle)

if (nargin < 2)
  fig = figure(20);
else
  fig = figure(figHandle);
end



if ~isfield(inputData, 'plotWeights')
  inputData.plotWeights = false;
end

numPlots = 2;
if inputData.plotWeights
  numPlots = 3;
end

numPlots = 3;


clf(fig);
set(fig,'Visible','off');
numIdx = length(inputData.times);

particleFilter = inputData.particleFilter;

sld = uicontrol('Style', 'slider', 'Min', 1, 'Max', numIdx, ...
    'Value', 1, 'Callback', @showPlot, 'Position', [400 20 120 20]);

set(fig,'Visible', 'on')


  function showPlot(source, event)

    idx = floor(get(source,'Value'));
    
    t = inputData.times(idx);
    subplot(numPlots,1,1)
    cla reset;
    hold on;

    % always overlay the nominal trajectory in the background
    particleFilter.plotNominalTraj();
    
    % plot particle set
    if (isfield(inputData,'particleSetArray'))
      particleSet = inputData.particleSetArray{idx};
      particleFilter.plotParticleSet(particleSet, fig);
    end

    % plot true particle
    if (isfield(inputData,'trueParticleArray'))
      particleFilter.plotSingleParticle(inputData.trueParticleArray{idx}, struct('colorString','g'));
    end


    if (isfield(inputData, 'kalmanFilterParticleArray'))
      inputData.ekf.plotKalmanFilterState(inputData.kalmanFilterParticleArray{idx});
    end

    titleString = strcat('Left leg, t = ', num2str(t));
    title(titleString);
    hold off;


    subplot(numPlots,1,2)
    cla reset;
    hold on;
    idx = floor(get(source,'Value'));
    t = inputData.times(idx);
    options = struct();
    options.plotType = 'right';
    particleFilter.plotNominalTraj();

    if (isfield(inputData,'particleSetArray'))
      particleSet = inputData.particleSetArray{idx};
      particleFilter.plotParticleSet(particleSet, fig, options);
    end

    if (isfield(inputData,'trueParticleArray'))
      trueOptions = options;
      trueOptions.colorString = 'g';
      particleFilter.plotSingleParticle(inputData.trueParticleArray{idx}, trueOptions);
    end


    if (isfield(inputData, 'kalmanFilterParticleArray'))
      inputData.ekf.plotKalmanFilterState(inputData.kalmanFilterParticleArray{idx}, options);
    end

    titleString = strcat('Right Leg t = ', num2str(t));
    title(titleString);
    hold off;


    subplot(numPlots,1,3);
    cla reset;
    hold on;
    options.plotType = 'position';
    trueOptions = options;
    trueOptions.colorString = 'g';

    if (isfield(inputData,'trueParticleArray'))
      particleFilter.plotSingleParticle(inputData.trueParticleArray{idx}, trueOptions);
    end

    if (isfield(inputData, 'kalmanFilterParticleArray'))
      inputData.ekf.plotKalmanFilterState(inputData.kalmanFilterParticleArray{idx}, options);
    end

    if (isfield(inputData, 'observationArray'))
      yParticle = inputData.observationArray{idx};
      obsOptions = options;
      obsOptions.colorString = 'k';
      inputData.ekf.plotKalmanFilterState(yParticle,obsOptions);
      % scatter(y(1),y(2),'filled','MarkerEdgeColor',[1 0.5  0]);
    end
    xlabel('qL');
    ylabel('qR');
    title('observation plot in position space');
    hold off;

  end



end