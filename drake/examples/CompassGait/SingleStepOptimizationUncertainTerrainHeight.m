function [returnData]=SingleStepOptimizationUncertainTerrainHeight(xtraj_seed, options)


xtraj_seed = xtraj_seed.shiftTime(-xtraj_seed.tspan(1));

if nargin < 2
  options = struct();
end

% setup the default options
defaultOptions = struct();
defaultOptions.numKnotPoints = 20;
defaultOptions.slope = 0;
defaultOptions.periodic = true;
defaultOptions.u_const_across_transitions = true;
defaultOptions.u_const_across_transitions_negate_sign = true;
defaultOptions.stanceLegSweepAngleLowerBound = 0.2;
defaultOptions.plot = true;
defaultOptions.useDeltaUCost = false;
defaultOptions.deltaUCostWeight = 1.0;
defaultOptions.robustCostFunction.Q = eye(4);
defaultOptions.robustCostFunction.R = 1;


options = applyDefaults(options, defaultOptions);


if ~isfield(options, 'plant')
  p = CompassGaitPlant();
else
  p = options.plant;
end


% number of knot points in the optimization
N = options.numKnotPoints;

duration = [0.4, 2.0];
stancePlant = p.modes{1};
traj_opt = UncertainTerrainHeightOptimization(p, N, duration, options);

costFunctionOptions = struct();
costFunctionOptions.Q = eye(4);
costFunctionOptions.R = 1;
traj_opt = traj_opt.setupOptions(costFunctionOptions);

x0 = [0;0;2;-.4];
t1 = .417;
x1 = [-.326;.22;-.381;-1.1];
tf = .713;
xf = x0;

% initial guess for timesteps
t_init = linspace(0,tf,N);

%initial guess for trajectories
traj_init = struct();
traj_init.x = xtraj_seed;
traj_init.u = ConstantTrajectory(0);

% linear constraint to control walking speed. Put a constraint on how the angle that the stance leg sweeps during one step.
stanceLegInStateVarIdx = 2;
stanceLegInitialIdx = traj_opt.x_inds(stanceLegInStateVarIdx, 1);
stanceLegFinalIdx = traj_opt.x_inds(stanceLegInStateVarIdx, end);

xinds = [stanceLegInitialIdx, stanceLegFinalIdx];

lb = options.stanceLegSweepAngleLowerBound;
ub = Inf; % this is what controls the speed, is binding in general for flat ground optimization
A = [1,-1];
stanceLegSweepAngleConstraint = LinearConstraint(lb,ub,A);
traj_opt = traj_opt.addConstraint(stanceLegSweepAngleConstraint, xinds);


%% add the running cost
traj_opt = traj_opt.addRunningCost(@CompassGaitTrajectoryOptimizationUtils.controlInputQuadraticCost);

%% add deltaU cost
weight = 3;
traj_opt = traj_opt.addDeltaUCost(weight);

%% Add a second trajectory
gamma_early = 2*pi/180;
p_early = CompassGaitPlant(gamma_early);
% [traj_opt, traj_idx] = traj_opt.addTrajectoryDecisionVariables(N, p_early);
% traj_opt = traj_opt.addTrajectoryDynamicsConstraints(traj_idx);

% add reset constraint for second trajectory

%% add in the hybrid transition and periodicity constraints
data.xm_idx = traj_opt.x_inds(:,end);
data.xp_idx = traj_opt.x_inds(:,1);
traj_opt = traj_opt.addResetMapConstraint(data);



%% hybrid guard constraints
traj_idx = 0;

% add guard constraints for the early reset thing

if true
  N_early = N - 2;
  guardOptions = struct();
  guardOptions.hybridPlant = p_early;
  guardOptions.knot_points = 1:N_early;
  guardOptions.equality_constraint_on_last_knot_point = true;
  traj_opt = traj_opt.addGuardConstraints(traj_idx, guardOptions);
else
  N_early = 0;
end

guardOptions = struct();
guardOptions.hybridPlant = p;
guardOptions.knot_points = N_early+1:N;
guardOptions.equality_constraint_on_last_knot_point = true;
traj_opt = traj_opt.addGuardConstraints(traj_idx, guardOptions);


% Add guard constraints for early reset trajecotry
% traj_idx = 1;
% % use the default plant for doing this
% traj_opt = traj_opt.addGuardConstraints(traj_idx);


%% Add terrain uncertainty cost function
% traj_opt = traj_opt.addUncertainTerrainCost();



data = struct();
data.t_init = {};
data.traj_init = {};
tic
returnData = solveTraj(traj_opt,t_init,traj_init, data);
returnData.info % print out the info of the trajectory optimization, info = 1 is good.
toc

xtraj = returnData.xtraj;
utraj = returnData.utraj;

if (options.plot)
  v = CompassGaitVisualizer(p, xtraj.getOutputFrame);
  figure(1); clf;
  fnplt(utraj);
  title('utraj');
  
  figure(2); clf; hold on;
  fnplt(xtraj,[1 3]);
  fnplt(xtraj,[2 4]);
  h = fnplt(xtraj_seed,[1 3]);
  set(h,'Color','r');
  h = fnplt(xtraj_seed,[2 4]);
  set(h,'Color','r');
  title('xtraj');

  figure(3); clf; hold on;
  fnplt(xtraj, [2,1])
  title('qtraj')
  xlabel('stance angle')
  ylabel('swing angle')
  hold off;
  
  playback(v,xtraj, struct('slider', true));
end

end


% function [f,df] = transition_constraint_fun_lucas(transition_fun,xm,u,xp)
% init_mode = 0;
% t = 0;
% [xp_fun,~,~,dxp] = transition_fun(init_mode,t,xm,u);
% f = xp - xp_fun;
% df = [-dxp(:,3:end) eye(length(xp))];
% end

% function [f,df] = guard_constraint_fun(p, guard_fun,x,u)
% [f,dg] = guard_fun(p, 0,x,u);
% df = dg(:,2:end);
% end



% function [g,dg] = cost(t,x,u);
% R = 1;
% g = sum((R*u).*u,1);
% dg = [zeros(1,1+size(x,1)),2*u'*R];
% end


% function [h,dh] = finalcost(t,x)
% h=t;
% dh = [1,zeros(1,size(x,1))];
% end

% function J = postImpactTrajCost(T,X,U,p)
% % encourage post-impact trajectory to leave collision surface orthogonally
% t0=T(1); x0=X(:,1); u0=U(:,1);
% xdot0=p.modes{2}.dynamics(t0,x0,u0);
% dphidx = [1,1,0,0]; % gradient of foot collision guard 1 w/respect to x
% J=100*abs(dphidx*xdot0)./norm(dphidx)./norm(xdot0);
% end




