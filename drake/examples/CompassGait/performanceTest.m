%% Get trajectory of passive Compass Gait
% This is  in order to a pass a reasonable initial trajectory to the trajectory optimization
% First we simulate a passive compass gait on a slope
gammaIn = 3*pi/180;
r = CompassGaitPlant(gammaIn);

x0 = r.getInitialState;
T = 10;


%% simulation without feedback
[ytraj, xtraj] = simulate(r, [0, T], x0);
v = CompassGaitVisualizer(r);


%% Setup the utilities
plant = TimeSteppingCompassGaitPlant(gammaIn);
cgUtils = CompassGaitUtils();

t_0 = 0;
x_init = xtraj.eval(t_0);

t_f_vec = [0.01,0.02,0.05,0.1,0.3];

for i=1:length(t_f_vec)
  disp('')
  disp('------------')
  t_f = t_f_vec(i)
  tspan = [t_0, t_f]
  [t,y] = plant.simulateWithConstantControlInputODE(x_init(2:end),tspan,0);
  numTimesteps_ode45 = numel(t)
  [t,y] = plant.simulateWithConstantControlInputODE4(x_init(2:end),tspan,0);
  numTimesteps_ode4 = numel(t)  
  disp('--------------')
  disp('')
end