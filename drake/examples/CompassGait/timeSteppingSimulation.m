gammaIn = 3*pi/180;
r = TimeSteppingCompassGaitPlant(gammaIn);

x_initial = r.getInitialState();
tspan = [0,2.0];


[t,y, te, ye, ie] = r.simulateWithConstantControlInputODE(x_initial(2:end), tspan, 0);

xtraj = PPTrajectory(pchip(t,y'));


s = CompassGaitPlant(gammaIn);
v = CompassGaitVisualizer(r, xtraj.getOutputFrame);
v.playback(xtraj, struct('slider', true));

%% Test our timestepping style simulation



controller = SimpleController(r);
dt = 0.01;
startTime = tic;
d = runTimeSteppingSimulationWithController(r, controller, tspan, dt, x_initial); 
elapsedTime = toc(startTime)
v = CompassGaitVisualizer(r, d.xtraj.getOutputFrame);
v.playback(d.xtraj, struct('slider',true));


%%
% 
% odefun = @(t,y) y;
% y0 = 1;
% tspan = [0,1];
% 
% [t,y] = 