gammaIn = 3*pi/180;
r = TimeSteppingCompassGaitPlant(gammaIn);

x_initial = r.getInitialState();
tspan = [0,10];

tic;
[ytraj, xtraj] = r.simulateWithConstantControlInput(x_initial, tspan, 0);
toc
v = CompassGaitVisualizer(r);
v.playback(ytraj, struct('slider', true))

controller = SimpleController(r);
dt = 0.05;
tic;
d = runTimeSteppingSimulationWithController(r, controller, tspan, dt, x_initial); 
toc
v.playback(d.ytraj, struct('slider',true));