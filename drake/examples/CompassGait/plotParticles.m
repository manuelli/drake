% interactive window for plotting the particles
% inputData should have fileds
% - particleFilter
% - particleSetArray
% - times
% - trueParticleArray
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


clf(fig);
set(fig,'Visible','off');
numIdx = length(inputData.times);

particleFilter = inputData.particleFilter;

sld = uicontrol('Style', 'slider', 'Min', 1, 'Max', numIdx, ...
    'Value', 1, 'Callback', @showPlot, 'Position', [400 20 120 20]);

set(fig,'Visible', 'on')


  function showPlot(source, event)

    idx = floor(get(source,'Value'));
    particleSet = inputData.particleSetArray{idx};
    t = inputData.times(idx);
    subplot(numPlots,1,1)
    cla reset;
    
    
    particleFilter.plotParticleSet(particleSet, fig);
    particleFilter.plotSingleParticle(inputData.trueParticleArray{idx}, struct('colorString','g'));
    titleString = strcat('Left leg, t = ', num2str(t));
    title(titleString);


    subplot(numPlots,1,2)
    cla reset;
    idx = floor(get(source,'Value'));
    t = inputData.times(idx);
    options = struct();
    options.plotRightLeg = true;
    particleFilter.plotParticleSet(particleSet, fig, options);
    particleFilter.plotSingleParticle(inputData.trueParticleArray{idx}, struct('colorString','g', 'plotRightLeg',true));
    titleString = strcat('Right Leg t = ', num2str(t));
    title(titleString);


    if inputData.plotWeights
      subplot(numPlots,1,3);
      cla reset;
      particleFilter.plotParticleSetWeights(particleSet)
    end
  end



end