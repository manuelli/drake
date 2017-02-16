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
defaultOptions.use_gamma_min_max = true;
defaultOptions.gamma_early = 2;
defaultOptions.gamma_late = 4;
defaultOptions.gamma_nominal = 3;
defaultOptions.add_delta_t_cost = true;
defaultOptions.delta_t_cost_weight = 0.1;


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
costFunctionOptions.uncertainTerrainHeightWeight = 1;
traj_opt = traj_opt.setupOptions(costFunctionOptions);

x0 = [0;0;2;-.4];
t1 = .417;
x1 = [-.326;.22;-.381;-1.1];
tf = .713;
xf = x0;

% initial guess for timesteps
t_init = linspace(xtraj_seed.tspan(1), xtraj_seed.tspan(2),N);

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
% traj_opt = traj_opt.addRunningCost(@CompassGaitTrajectoryOptimizationUtils.controlInputQuadraticCost);
traj_opt = traj_opt.addRunningCost(@CompassGaitTrajectoryOptimizationUtils.controlInputQuadraticCostIntegrated);

%% add deltaU cost
if options.useDeltaUCost
  traj_opt = traj_opt.addDeltaUCost(options.deltaUCostWeight);
end

if options.add_delta_t_cost
  traj_opt = traj_opt.addDeltaTCost(options.delta_t_cost_weight);
end

%% experimental for now
p_early = CompassGaitPlant(options.gamma_early*pi/180);
p_late = CompassGaitPlant(options.gamma_late*pi/180);

N_nom = options.N_nom;

if options.earlyResetGuard
  N_early = options.N_early;
else
  N_early = 0;
end

if options.lateResetGuard
  N_late = options.N_late;
else
  N_late = N_nom;
end

% early reset guard
guardOptions = struct();
guardOptions.hybridPlant = p_early;
guardOptions.knot_points = 1:N_early;
guardOptions.equality_constraint_on_last_knot_point = true;
traj_opt = traj_opt.addGuardConstraints(0, guardOptions);


% nominal reset guard
guardOptions = struct();
guardOptions.hybridPlant = p;
guardOptions.knot_points = N_early+1:N_nom;
guardOptions.equality_constraint_on_last_knot_point = true;
traj_opt = traj_opt.addGuardConstraints(0, guardOptions);


% late reset guard
guardOptions = struct();
guardOptions.hybridPlant = p_late;
guardOptions.knot_points = N_nom+1:N_late;
guardOptions.equality_constraint_on_last_knot_point = true;
traj_opt = traj_opt.addGuardConstraints(0, guardOptions);



%% add in the hybrid transition and periodicity constraints
data.xm_idx = traj_opt.x_inds(:,N_nom);
data.xp_idx = traj_opt.x_inds(:,1);
traj_opt = traj_opt.addResetMapConstraint(data);

% %% Add a second trajectory
% gamma_early = 2*pi/180;
% p_early = CompassGaitPlant(gamma_early);

% % add guard constraints for the early reset thing
% if true
%   N_early = N - 2;
%   guardOptions = struct();
%   guardOptions.hybridPlant = p_early;
%   guardOptions.knot_points = 1:N_early;
%   guardOptions.equality_constraint_on_last_knot_point = true;
%   traj_opt = traj_opt.addGuardConstraints(0, guardOptions);
% else
%   N_early = 0;
% end

% early_reset_traj_active = false;
% if early_reset_traj_active
%   [traj_opt, traj_idx] = traj_opt.addTrajectoryDecisionVariables(N, p_early);
%   traj_opt = traj_opt.addTrajectoryDynamicsConstraints(traj_idx);

%   % add reset constraint for second trajectory
%   data.xm_idx = traj_opt.x_inds(:,N_early);
%   data.xp_idx = traj_opt.xtraj_inds{traj_idx}(:,1);
%   traj_opt = traj_opt.addResetMapConstraint(data);

%   % guard constraints for additional trajectory
%   traj_opt = traj_opt.addGuardConstraints(traj_idx);

%   data = struct();
%   data.traj_idx = traj_idx;
%   traj_opt = traj_opt.addRunningCostForTrajectory(data);
% end


%% Add terrain uncertainty cost function

% this version is too slow an unusable.
% traj_opt = traj_opt.addUncertainTerrainCost();

% simpler version
% % num_knot_points_for_cost = floor(options.numKnotPoints/4);
% num_knot_points_for_cost = 1;
% data = struct();
% data.Q = eye(4);
% data.traj_idx = 1;
% data.nominal_traj_knot_points = [N - num_knot_points_for_cost + 1:N];
% data.alternate_traj_knot_points = [N - num_knot_points_for_cost + 1:N];

% add cost
% traj_opt = traj_opt.addTrajectoryDeviationCost(data);

% % add equality constraint on last knot point.
% x_ind_1 = traj_opt.x_inds(:,N);
% x_ind_2 = traj_opt.xtraj_inds{1}(:,N);
% traj_opt = traj_opt.addEqualityConstraint(x_ind_1, x_ind_2);



data = struct();
data.t_init = {};
data.traj_init = {};

early_reset_traj_active = false;
if early_reset_traj_active
  traj_idx = 1;
  data.t_init{traj_idx} = t_init;
  data.traj_init{traj_idx} = traj_init;
end


tic
returnData = solveTraj(traj_opt,t_init,traj_init, data);
disp('info = ')
returnData.info % print out the info of the trajectory optimization, info = 1 is good.
toc

xtraj = returnData.xtraj;
utraj = returnData.utraj;

if early_reset_traj_active
  xtraj_early = returnData.xtraj_idx{1};
  utraj_early = returnData.utraj_idx{1};
end

if (options.plot)
  v = CompassGaitVisualizer(p, xtraj.getOutputFrame);
  figure(1); clf;
  hold on;
  fnplt(utraj);
  title('utraj');
  
  if early_reset_traj_active
    h = fnplt(utraj_early);
    set(h,'Color','g');
  end
  
  hold off;
  
  figure(2); clf; hold on;
  fnplt(xtraj,[1 3]);
  fnplt(xtraj,[2 4]);
  h = fnplt(xtraj_seed,[1 3]);
  set(h,'Color','r');
  h = fnplt(xtraj_seed,[2 4]);
  set(h,'Color','r');
  
  
  if early_reset_traj_active
    disp('plotting early reset trajectory')
    h = fnplt(xtraj_early,[1 3]);
    set(h,'Color','g');
    h = fnplt(xtraj_early,[2 4]);
    set(h,'Color','g');
  end
  
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




