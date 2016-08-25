% test dircolCycleLucas

%% Run trajectory optimization

options = struct();
options.slope = 0;
options.numKnotPoints = 50;
options.plot = true;
options.u_const_across_transitions = false;

[p,utraj,xtraj,z,traj_opt]=runDircolCycleLucas(options);

utrajPhase = returnData.utrajPhase;


v = CompassGaitVisualizer(p, xtraj.getOutputFrame);
v.playback(xtraj, struct('slider',true));


%% Plot some information
tBreaks = xtraj.getBreaks();
fig = figure(1);
clf(fig);
hold on;
plot(tBreaks, utraj.eval(tBreaks));
hold off;



%% Test plotting

% % just try to get a sense of what the trajectory looks like
% x = xtraj.traj{1}.trajs{2}; % the first part of the x trajectory
% xDeriv = x.fnder(1);
% xDeriv2 = x.fnder(2);
% tGrid = linspace(x.tspan(1), x.tspan(2), 100);
% xGrid = x.eval(tGrid);
% xDerivGrid = xDeriv.eval(tGrid);
% xDeriv2Grid = xDeriv2.eval(tGrid);
% 
% tBreaks = x.getBreaks();
% xTrueGrid = x.eval(tBreaks);
% xDerivTrueGrid = xDeriv.eval(tBreaks);
% hDerivTrueGrid = xTrueGrid(3,:)./xTrueGrid(4,:);
% 
% 
% % hDeriv2TrueGrid = xDerivTrueGrid(3,:)./xTrueGrid(4,:).^2 - ...
% %   xTrueGrid(3).*xDerivTrueGrid(4,:)/(xTrueGrid(4,:).^3)
% 
% hDeriv2TrueGrid = xDerivTrueGrid(3,:)./xTrueGrid(4,:).^2 - ...
%   hDerivTrueGrid.*(xDerivTrueGrid(4,:))./xTrueGrid(4,:).^2;
% 
% hDerivTraj = PPTrajectory(pchip(tBreaks, hDerivTrueGrid));
% hDeriv2Traj = PPTrajectory(pchip(tBreaks, hDeriv2TrueGrid));
% 
% hDerivPlotGrid = hDerivTraj.eval(tGrid);
% hDeriv2PlotGrid = hDeriv2Traj.eval(tGrid);
% 
% hDerivPhaseTraj = PPTrajectory(pchip(xGrid(2,:), hDerivTraj.eval(tGrid)));
% hDeriv2PhaseTraj = PPTrajectory(pchip(xGrid(2,:), hDeriv2Traj.eval(tGrid)));
% 
% 
% fig = figure(1);
% clf(fig);
% hold on;
% plot(tGrid, xGrid(1,:),'b');
% plot(tGrid, xGrid(2,:), 'r');
% plot(tGrid, xGrid(3,:),'b--');
% plot(tGrid, xGrid(4,:), 'r--');
% % plot(tGrid, xDerivGrid(1,:), 'g');
% 
% hold off;
% 
% 
% fig = figure(2);
% clf(fig);
% hold on;
% plot(tGrid, xDerivGrid(3,:),'b');
% plot(tGrid, xDeriv2Grid(4,:), 'r');
% plot(tGrid, hDerivPlotGrid, 'g');
% % plot(tGrid, xGrid(3,:),'b--');
% % plot(tGrid, xGrid(4,:), 'r--');
% hold off;
% 
% 
% fig = figure(3);
% clf(fig);
% hold on
% plot(tGrid, hDerivPlotGrid, 'b');
% plot(tGrid, hDeriv2PlotGrid, 'r');
% hold off
% 
% 
% thetaGrid = xGrid(2,:);
% 
% fig = figure(4);
% clf(fig);
% hold on
% xPhaseGrid = xtrajPhase.eval(thetaGrid);
% plot(thetaGrid, xPhaseGrid(2,:),'r');
% plot(thetaGrid, xGrid(1,:),'b');
% hold off
% 
% 
% fig = figure(5);
% clf(fig);
% hold on
% xPhaseDerivGrid = xtrajPhaseDeriv.eval(thetaGrid);
% plot(thetaGrid, hDerivPhaseTraj.eval(thetaGrid), 'b');
% plot(thetaGrid, hNumericDeriv.eval(thetaGrid), 'r');
% hold off
% 
% fig = figure(6);
% clf(fig);
% hold on;
% thetaGrid = linspace(-0.3,0.01,100);
% plot(thetaGrid, hDeriv2PhaseTraj.eval(thetaGrid), 'b');
% plot(thetaGrid, hNumericDeriv2.eval(thetaGrid), 'r');
% hold off;












































