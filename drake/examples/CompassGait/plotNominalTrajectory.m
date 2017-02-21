function plotNominalTrajectory(xtraj, gammaVals)
  fig = figure(3); clf; hold on;
  fnplt(xtraj, [2,1]);
  for i=1:length(gammaVals)
    gammaInDegrees = gammaVals(i);
    plotResetBoundary(gammaInDegrees);
  end
  title('qtraj');
  xlabel('stance angle')
  ylabel('swing angle')
  hold off;
end