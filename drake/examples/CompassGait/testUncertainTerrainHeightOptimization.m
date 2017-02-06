function testUncertainTerrainHeightTrajectoryOptimization()

p = CompassGaitPlant();
duration = [0.4, 2.0];
stancePlant = p.modes{1};
N = 30;
% traj_opt = UncertainTerrainHeightOptimization(p, N, duration);
traj_opt = DircolTrajectoryOptimization(stancePlant, N, duration);



% %% Add variables and dynamics constraints
% if false
%   traj_opt = traj_opt.addTrajectoryDecisionVariables(N, p);
%   traj_idx = 1;
%   traj_opt = traj_opt.addTrajectoryDynamicsConstraints(traj_idx);
% end

% % reset constraint
% data.xp_idx = traj_opt.x_inds(:,1);
% data.xm_idx = traj_opt.x_inds(:,end);
% traj_opt = traj_opt.addResetMapConstraint(data);
transition_fun = @(xm,u,xp) transition_constraint_fun_lucas(@p.collisionDynamics,xm,u,xp);

numStates = stancePlant.getNumStates();
numInputs = stancePlant.getNumInputs();
jump_con = FunctionHandleConstraint(zeros(numStates,1), zeros(numStates,1), 2*numStates + numInputs, transition_fun);

jump_xind{1} = traj_opt.x_inds(:,end); % xm = x_inds for the last knot point
jump_xind{2} = traj_opt.u_inds(:,end); % u
jump_xind{3} = traj_opt.x_inds(:,1); % xp = x_inds for the first knot point

traj_opt = traj_opt.addConstraint(jump_con, jump_xind);

% % guard constraint
% traj_idx = 0;
% traj_opt = traj_opt.addGuardConstraints(traj_idx);
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

% %% Add the costs
% traj_opt = traj_opt.addRunningCost(@CompassGaitTrajectoryOptimizationUtils.controlInputQuadraticCost);
traj_opt = traj_opt.addRunningCost(@cost);




% % % Add delta u cost
% weight = 10;
% traj_opt = traj_opt.addDeltaUCost(weight);

%% Get seed
gammaSeed = 3*pi/180;
r = CompassGaitPlant(gammaSeed);
x0 = r.getInitialState;
T = 10.0;
[ytraj, xtraj] = simulate(r, [0, T], x0);
xtraj_single_step = xtraj.traj{8};

% removes hybrid mode from trajectory
xtraj_seed = xtraj_single_step(2:5);

x0 = [0;0;2;-.4];
t1 = .417;
x1 = [-.326;.22;-.381;-1.1];
tf = xtraj_seed.tspan(2) - xtraj_seed.tspan(1);
xf = x0;

% initial guess for timesteps
t_init = linspace(0,tf,N);

%initial guess for trajectories
traj_init = struct();
traj_init.x = xtraj_seed;
traj_init.u = ConstantTrajectory(0);

disp('starting optimization');


tic;
[xtraj_opt,utraj,z,F,info] = solveTraj(traj_opt,t_init,traj_init);
info
toc

%% Testing
xp = xtraj_opt.eval(0)
xm = xtraj_opt.eval(xtraj.tspan(2));

xp_test = p.collisionDynamics(1,0,xm,0)


t = 1.12;
x = xtraj_opt.eval(t);

p.guard
















%% Plotting
xtraj = xtraj_opt;
if (true)
  v = CompassGaitVisualizer(p, xtraj.getOutputFrame);
  figure(1); clf;
  fnplt(utraj);
  title('utraj');
  
  figure(2); clf; hold on;
  fnplt(xtraj,[1 3]);
  fnplt(xtraj,[2 4]);
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


%% Testing
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



function [g,dg] = cost(t,x,u)
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

