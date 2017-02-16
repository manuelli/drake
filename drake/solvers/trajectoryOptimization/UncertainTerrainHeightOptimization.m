classdef UncertainTerrainHeightOptimization < DircolTrajectoryOptimization

properties
traj_names = {};
xtraj_inds = {};
utraj_inds = {};
htraj_inds = {};
hybridPlant_array = {};
N_j = {};
nX; % num states
nU; % num control inputs
hybridPlant;
costFunctionOptions;
end

methods
  function obj = UncertainTerrainHeightOptimization(hybridPlant, N, duration, varargin)
    obj = obj@DircolTrajectoryOptimization(hybridPlant.modes{1},N,duration,varargin{:});
    disp('using a UncertaintyTerrainHeight optimizer')
    obj.nX = obj.plant.getNumStates();
    obj.nU = obj.plant.getNumInputs();
    obj.hybridPlant = hybridPlant;
  end

  function obj = setupOptions(obj, options)
    defaultOptions = struct();
    defaultOptions.Q = eye(4);
    defaultOptions.R = 1;

    if nargin < 2
      options = struct();
    end

    obj.costFunctionOptions = applyDefaults(options, defaultOptions);
  end


  % adds variables for another trajectory
  function [obj, idx] = addTrajectoryDecisionVariables(obj, num_knot_points, hybridPlant)
    num_new_state_vars = obj.nX*num_knot_points;
    num_new_control_vars = obj.nU*num_knot_points;

    num_new_time_vars = num_knot_points - 1;
    [obj, new_variable_idx] = obj.addDecisionVariable(num_new_time_vars);
    new_time_variable_idx = new_variable_idx;

    % add constraint that these time variables be non-negative
    obj = obj.addConstraint(BoundingBoxConstraint(zeros(num_new_time_vars,1),inf(num_new_time_vars,1)),new_time_variable_idx);

    % add additional state variables
    [obj, new_variable_idx] = obj.addDecisionVariable(num_new_state_vars);
    new_state_variable_idx = reshape(new_variable_idx, [obj.nX, num_knot_points]);

    
    % add additional control variables
    [obj, new_variable_idx] = obj.addDecisionVariable(num_new_control_vars);
    new_control_variable_idx = reshape(new_variable_idx, [obj.nU, num_knot_points]);

    % record indices in object
    idx = length(obj.N_j) + 1;
    obj.N_j{idx} = num_knot_points;
    obj.htraj_inds{idx} = new_time_variable_idx;
    obj.xtraj_inds{idx} = new_state_variable_idx;
    obj.utraj_inds{idx} = new_control_variable_idx;
    obj.hybridPlant_array{idx} = hybridPlant;
  end

  function obj = addTrajectoryDynamicsConstraints(obj, traj_idx)
    xinds = obj.xtraj_inds{traj_idx};
    uinds = obj.utraj_inds{traj_idx};
    hinds = obj.htraj_inds{traj_idx};
    N = obj.N_j{traj_idx};

    nX = obj.plant.getNumStates();
    nU = obj.plant.getNumInputs();
    constraints = cell(N-1,1);
    dyn_inds = cell(N-1,1);
    
    
    n_vars = 2*nX + 2*nU + 1;
    cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.constraint_fun);
    cnstr = setName(cnstr,'collocation');

    shared_data_index = obj.getNumSharedDataFunctions;
      for i=1:N,
        obj = obj.addSharedDataFunction(@obj.dynamics_data,{xinds(:,i);uinds(:,i)});
      end
      
      for i=1:N-1,
        dyn_inds{i} = {hinds(i);xinds(:,i);xinds(:,i+1);uinds(:,i);uinds(:,i+1)};
        constraints{i} = cnstr;
        
        obj = obj.addConstraint(constraints{i}, dyn_inds{i},[shared_data_index+i;shared_data_index+i+1]);
      end
  end

  % data should be a struct, with fields
  % xm_idx
  % xp_idx
  function obj = addResetMapConstraint(obj, data)
    disp('')
    disp('---------------------------')
    disp('adding reset map constraint')
    eval_handle = @(xm,xp) obj.hybridResetMap(xm,xp);
    reset_constraint = FunctionHandleConstraint(zeros(obj.nX,1),zeros(obj.nX,1), 2*obj.nX, eval_handle);
    reset_constraint_inds{1} = data.xm_idx;
    reset_constraint_inds{2} = data.xp_idx;
    obj = obj.addConstraint(reset_constraint, reset_constraint_inds);
  end


  function [f,df] = hybridResetMap(obj,xm,xp)
    u_hack = 0; % this is irrelevant
    init_mode = 0; % this is irrelevant
    t_hack = 0;
    [xp_fun,~,~,dxp] = obj.hybridPlant.collisionDynamics(init_mode,t_hack,xm,u_hack);
    f = xp - xp_fun;

    % need to be careful to grab only the relevant indices
    start_idx = 3;
    end_idx = 3+obj.nX -1;
    df = [-dxp(:,start_idx:end_idx) eye(length(xp))];
  end

  % only adds it on the nominal trajectory
  function obj = addDeltaUCost(obj, weight)
    Q = weight*[1,;-1]*[1,-1];
    b = zeros(2,1);
    for i=1:obj.N-1
      uind = [obj.u_inds(1,i); obj.u_inds(1,i+1)];
      obj = obj.addCost(QuadraticConstraint(0,inf,Q,b), uind);
    end
  end

  % only adds it on the nominal trajectory
  % same as above but multiplies cost by dt * dU^2
  function obj = addDeltaUCostIntegrated(obj, weight)
    Q = weight*[1,;-1]*[1,-1];
    b = zeros(2,1);
    for i=1:obj.N-1
      uind = [obj.u_inds(1,i); obj.u_inds(1,i+1)];
      obj = obj.addCost(QuadraticConstraint(0,inf,Q,b), uind);
    end
  end


  function obj = addDeltaTCost(obj, weight)
    Q = weight;
    b = 0;
    for i=1:obj.N-1
      hind = obj.h_inds(i);
      obj = obj.addCost(QuadraticConstraint(0,inf,Q,b), hind);
    end
  end

  % this only applies to the nominal trajecotry
  function obj = addUConstAcrossTransitionsConstraint(obj)
    if options.u_const_across_transitions
      uind = [obj.u_inds(1,1); obj.u_inds(1,end)];
      Q = options.deltaUCostWeight*ones(2,2);
      obj = obj.addCost(QuadraticConstraint(0,inf,Q,b), uind);
    end
  end

  % if traj_idx = 0 then it is the nominal trajectory
  function obj = addGuardConstraints(obj, traj_idx, options)
    if (traj_idx == 0)
      xinds = obj.x_inds;
      uinds = obj.u_inds;
      hybridPlant = obj.hybridPlant;
      N = obj.N;
    else
      xinds = obj.xtraj_inds{traj_idx};
      uinds = obj.utraj_inds{traj_idx};
      hybridPlant = obj.hybridPlant;
      N = obj.N_j{traj_idx};
    end

    if nargin < 3
      options = struct();
      options.hybridPlant = hybridPlant;
      options.knot_points = 1:N;
      options.equality_constraint_on_last_knot_point = true;
    end

    % they should all use the default hybrid plant
    guard_fun = @(x) obj.guardFunction(options.hybridPlant, x);
    for idx=1:length(options.knot_points)
      i = options.knot_points(idx);
      guard_lb = 0;
      guard_ub = Inf;

      % guard should be >= 0 except for last knot point where it should be equal to zero
      if (idx==length(options.knot_points) && options.equality_constraint_on_last_knot_point)
        disp('')
        disp('----------------')
        disp('equality guard constraint')
        i
        traj_idx
        guard_ub = 0;
      end

      guard_constraint = FunctionHandleConstraint(guard_lb, guard_ub, obj.nX, guard_fun);
      guard_xind{1} = xinds(:,i);
      obj = obj.addConstraint(guard_constraint, guard_xind);
    end
  end

  % Evaluates the guard function
  function [f,df] = guardFunction(obj, hybridPlant, x)
    t_hack = 0;
    u_hack = 0;
    [f,dg] = hybridPlant.guard{1}{1}(hybridPlant, t_hack,x,u_hack);
    % be careful with indices, only want to return gradient relevant to x
    % by default it is including everything
    start_idx = 2;
    end_idx = start_idx+obj.nX-1;
    df = dg(:,start_idx:end_idx);
  end

  function obj = addUncertainTerrainCost(obj)
    nonlinear_cost = FunctionHandleObjective(obj.num_vars, @(z)obj.uncertainTerrainHeightCostFunction(z));

    % tell the optimizer to user numerical gradients
    nonlinear_cost.grad_method = 'numerical';
    obj = obj.addCost(nonlinear_cost);
  end

  % vars are ALL the decision vars
  function costVal = uncertainTerrainHeightCostFunction(obj, vars)
    costVal = 0;
    t = [0; cumsum(vars(obj.h_inds))];
    x_nom = reshape(vars(obj.x_inds),[],obj.N);
    u_nom = reshape(vars(obj.u_inds),[],obj.N);

    numTerrainHeights = length(obj.xtraj_inds);
    xJ = {};
    uJ = {};
    hJ = {};
    for i=1:numTerrainHeights
      xJ{i} = reshape(vars(obj.xtraj_inds{i}),[],obj.N_j{i});
      uJ{i} = reshape(vars(obj.utraj_inds{i}),[],obj.N_j{i});
      hJ{i} = vars(obj.htraj_inds{i});
    end

    stanceLegIdx = 2;
    % tau_plus = x_nom(stanceLegIdx, 1); % this is positive usually
    % tau_minus = x_nom(stanceLegIdx, 0); % this is negative
    theta_max = max(x_nom(stanceLegIdx,:));
    theta_min = min(x_nom(stanceLegIdx, :));
    xtraj_phase = PPTrajectory(foh(x_nom(stanceLegIdx,:), x_nom));
    utraj_phase = PPTrajectory(foh(x_nom(stanceLegIdx,:), u_nom));

    Q = obj.costFunctionOptions.Q;
    R = obj.costFunctionOptions.R;

    function tau = tauFromTheta(theta)
      tau = (theta - theta_min)/(theta_max - theta_min);
    end

    function theta = thetaFromTau(tau)
      theta = tau*(theta_max - theta_min) + theta_min;
    end

    for j=1:numTerrainHeights
      costValTemp = 0;
      theta_j_plus = xJ{j}(stanceLegIdx, 1);
      theta_j_minus = xJ{j}(stanceLegIdx, end);

      tau_j_plus = tauFromTheta(theta_j_plus);
      tau_j_minus = tauFromTheta(theta_j_minus);

      % only go out to N_j{i} - 1 so it's easy to add the cost function
      for n=1:obj.N_j{j}-2
        x = xJ{i}(:,n);
        u = uJ{i}(:,n);
        theta = x(stanceLegIdx);
        tau = tauFromTheta(theta);

        % adjust theta if necessary
        if (tau < 0)
          theta = thetaFromTau(0);
        elseif (tau > 1)
          theta = thetaFromTau(1);
        end

        % matching point in the nominal trajecotry.
        x_tilde = xtraj_phase.eval(theta);
        u_tilde = utraj_phase.eval(theta);
        deltaX = x - x_tilde;
        deltaU = u - u_tilde;

        dt = 0.5*(hJ{j}(n) + hJ{j}(n+1));
        deltaCost = dt *(deltaX'*Q*deltaX + deltaU'*R*deltaU);
        % the squared is due to the expression on page 20
        deltaCost = deltaCost * (tau-tau_j_plus)/(tau_j_minus - tau_j_plus);
        costValTemp = costValTemp + deltaCost;
      end

      % can also add the weight here if we want
      costVal = costVal + (tau_j_minus - tau_j_plus)*costValTemp;
    end

    costVal = obj.costFunctionOptions.uncertainTerrainHeightWeight*costVal;
  end

  function returnData = solveTraj(obj, t_init, traj_init, data)
    % everything else should be set to zero I believe.
    z0 = obj.getInitialVars(t_init, traj_init);

    % initialize all the variables we have
    for idx=1:length(data.t_init)
      z0(obj.htraj_inds{idx}) = diff(data.t_init{idx});
      z0(obj.xtraj_inds{idx}) = data.traj_init{idx}.x.eval(data.t_init{idx});
    end

    [z,F,info,infeasible_constraint_name] = obj.solve(z0);
    xtraj = obj.reconstructStateTrajectory(z);
    utraj = obj.reconstructInputTrajectory(z);

    returnData = struct();
    returnData.xtraj = xtraj;
    returnData.utraj = utraj;
    returnData.info = info;
    returnData.infeasible_constraint_name = infeasible_constraint_name;
    returnData.F = F;

    returnData.xtraj_idx = {};
    returnData.utraj_idx = {};

    for traj_idx=1:length(obj.xtraj_inds)
      [xtraj_temp, utraj_temp] = obj.reconstructStateAndInputTrajecotry(z, traj_idx);
      returnData.xtraj_idx{traj_idx} = xtraj_temp;
      returnData.utraj_idx{traj_idx} = utraj_temp;
    end

  end

  % data should have fields
  % - traj_idx
  % - nominal_traj_knot_points
  % - alternate_traj_knot_point
  % both knot point arrays should have the same length
  % - Q the weight matrix for the cost
  function obj = addTrajectoryDeviationCost(obj, data)
    if (length(data.nominal_traj_knot_points) ~= length(data.alternate_traj_knot_points))
      error('nominal_traj_knot_points and alternate_traj_knot_point must have the same length')
    end

    temp = [data.Q, -data.Q];
    Q = temp'*temp;
    sz = size(Q);
    b = zeros(sz(1),1);
    xJ = obj.xtraj_inds{data.traj_idx};
    for i=1:length(data.nominal_traj_knot_points)
      nominal_idx = data.nominal_traj_knot_points(i);
      alt_idx = data.alternate_traj_knot_points(i);

      x_nom_inds = obj.x_inds(:,nominal_idx);
      x_alt_inds = xJ(:,alt_idx);

      x_ind = [x_nom_inds; x_alt_inds];
      % x_ind{1} = x_nom_inds
      % x_ind{2} = x_alt_inds
      obj = obj.addCost(QuadraticConstraint(0,inf,Q,b), x_ind);
    end

  end

  function obj = addEqualityConstraint(obj, x_ind_1, x_ind_2)
    lb = zeros(4,1);
    ub = zeros(4,1);
    A = [eye(4),-eye(4)];
    xind = [x_ind_1; x_ind_2];
    cnstr = LinearConstraint(lb,ub, A);
    obj = obj.addConstraint(cnstr, xind);
  end

  function obj = addRunningCostForTrajectory(obj, data)
    R = 1;
    b = 0;
    uJ = obj.utraj_inds{data.traj_idx};

    for i=1:length(uJ)
      uind = uJ(i);
      obj = obj.addCost(QuadraticConstraint(0,inf,R,b), uind);
    end
  end

  function [xtraj, utraj] = reconstructStateAndInputTrajecotry(obj,z, traj_idx)
    N = obj.N_j{traj_idx};
    x = reshape(z(obj.xtraj_inds{traj_idx}),[],N);
    u = reshape(z(obj.utraj_inds{traj_idx}),[],N);
    t = [0; cumsum(z(obj.htraj_inds{traj_idx}))];

    xtraj = PPTrajectory(foh(t,x));
    xtraj = xtraj.setOutputFrame(obj.plant.getStateFrame);
    utraj = PPTrajectory(foh(t,u));
  end

end


end
