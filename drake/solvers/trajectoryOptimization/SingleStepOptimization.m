function [p,utraj,xtraj,z,traj_opt, returnData]=SingleStepOptimization(xtraj_seed, options)
% from an initial balancing condition, take one step and return to the
% balance upright.


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


options = applyDefaults(options, defaultOptions);


if ~isfield(options, 'plant')
  p = CompassGaitPlant();
else
  p = options.plant;
end


% gamma = options.slope*pi/180; % this is the ground height
% p = CompassGaitPlant();

% number of knot points in the optimization
N = options.numKnotPoints;

duration = [0.4, 2.0];
stancePlant = p.modes{1};
traj_opt = DircolTrajectoryOptimization(stancePlant, N, duration);

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
% constraint at first knot point
% traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint([0.05;-inf(3,1)],[0;inf(3,1)]),1);

% traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint([-inf;0.05;-inf(2,1)],[inf(4,1)]),1);


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


% add u constant across transitions constraint if it was specified
if (options.u_const_across_transitions)
  lb = 0;
  ub = 0;
  % they should be opposite signs since the torque in motor coordinates depends on 
  % which is the stance leg
  A = [1,1]; 
  uConstConstraint = LinearConstraint(lb,ub,A);

  uInitialIdx = traj_opt.u_inds(1,1);
  uFinallIdx = traj_opt.u_inds(1,end);

  uinds = [uInitialIdx, uFinallIdx];
  traj_opt = traj_opt.addConstraint(uConstConstraint, uinds);
end


% add the running cost
traj_opt = traj_opt.addRunningCost(@cost);

if options.useDeltaUCost
  disp('adding delta u cost');
  Q = options.deltaUCostWeight*[1,;-1]*[1,-1];
  b = zeros(2,1);
  for i=1:options.numKnotPoints-1
    xind = [traj_opt.u_inds(1,i); traj_opt.u_inds(1,i+1)];
    traj_opt = traj_opt.addCost(QuadraticConstraint(0,inf,Q,b), xind);
  end
  
  % if we are also using this cost, then add a special one for final
  if options.u_const_across_transitions
    xind = [traj_opt.u_inds(1,1); traj_opt.u_inds(1,end)];
    Q = options.deltaUCostWeight*ones(2,2);
    traj_opt = traj_opt.addCost(QuadraticConstraint(0,inf,Q,b), xind);
  end
end


%% add in the hybrid transition and periodicity constraints
% first knot point should be what you get by applying reset map to the last
% knot point. Also need to include guard functions.
transition_fun = @(xm,u,xp) transition_constraint_fun_lucas(@p.collisionDynamics,xm,u,xp);


transition_constraint_fun_lucas(@p.collisionDynamics,x0,0,x0);

numStates = stancePlant.getNumStates();
numInputs = stancePlant.getNumInputs();
jump_con = FunctionHandleConstraint(zeros(numStates,1), zeros(numStates,1), 2*numStates + numInputs, transition_fun);

jump_xind{1} = traj_opt.x_inds(:,end); % xm = x_inds for the last knot point
jump_xind{2} = traj_opt.u_inds(:,end); % u
jump_xind{3} = traj_opt.x_inds(:,1); % xp = x_inds for the first knot point

traj_opt = traj_opt.addConstraint(jump_con, jump_xind);


%% Add guard function constraint at all the knot points

% this adds guard for the final knot point only
guard_lb = 0;

guard_ub = 0; % should be 0 if this is where transition is happening, otherwise it's infinity
guard_fun = @(x,u) guard_constraint_fun(p, p.guard{1}{1}, x, u);
guard_con = FunctionHandleConstraint(guard_lb, guard_ub, numStates + numInputs, guard_fun);

guard_xind{1} = traj_opt.x_inds(:,end);
guard_xind{2} = traj_opt.u_inds(:,end);
traj_opt = traj_opt.addConstraint(guard_con, guard_xind);

% add guard constraint for all other knot points
for k=1:N
  % this adds guard for the final knot point only
guard_lb = 0;
guard_ub = Inf;
if (k==N)
  guard_ub = 0; % should be 0 if this is where transition is happening, otherwise it's infinity
end

guard_fun = @(x,u) guard_constraint_fun(p, p.guard{1}{1}, x, u);
guard_con = FunctionHandleConstraint(guard_lb, guard_ub, numStates + numInputs, guard_fun);

guard_xind{1} = traj_opt.x_inds(:,k);
guard_xind{2} = traj_opt.u_inds(:,k);
traj_opt = traj_opt.addConstraint(guard_con, guard_xind);
end



% traj_opt = traj_opt.setCheckGrad(true);
% snprint('snopt.out');
tic
[xtraj,utraj,z,F,info] = solveTraj(traj_opt,t_init,traj_init);
info % print out the info of the trajectory optimization, info = 1 is good.
toc
if (options.plot)
  v = CompassGaitVisualizer(p, xtraj.getOutputFrame);
  figure(1); clf;
  fnplt(utraj);
  title('utraj');
  
  figure(2); clf; hold on;
  fnplt(xtraj,[1 3]);
  fnplt(xtraj,[2 4]);
  title('xtraj');
  
  playback(v,xtraj, struct('slider', true));
end


returnData = struct();
end


function [f,df] = transition_constraint_fun_lucas(transition_fun,xm,u,xp)
init_mode = 0;
t = 0;
[xp_fun,~,~,dxp] = transition_fun(init_mode,t,xm,u);
f = xp - xp_fun;
df = [-dxp(:,3:end) eye(length(xp))];
end

function [f,df] = guard_constraint_fun(p, guard_fun,x,u)
[f,dg] = guard_fun(p, 0,x,u);
df = dg(:,2:end);
end



function [g,dg] = cost(t,x,u);
R = 1;
g = sum((R*u).*u,1);
dg = [zeros(1,1+size(x,1)),2*u'*R];
end


function [h,dh] = finalcost(t,x)
h=t;
dh = [1,zeros(1,size(x,1))];
end

function J = postImpactTrajCost(T,X,U,p)
% encourage post-impact trajectory to leave collision surface orthogonally
t0=T(1); x0=X(:,1); u0=U(:,1);
xdot0=p.modes{2}.dynamics(t0,x0,u0);
dphidx = [1,1,0,0]; % gradient of foot collision guard 1 w/respect to x
J=100*abs(dphidx*xdot0)./norm(dphidx)./norm(xdot0);
end




