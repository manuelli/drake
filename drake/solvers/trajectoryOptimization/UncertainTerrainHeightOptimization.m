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
end

methods
  function obj = UncertainTerrainHeightOptimization(hybridPlant, N, duration, varargin)
    obj = obj@DircolTrajectoryOptimization(hybridPlant.modes{1},N,duration);
    disp('using a UncertaintyTerrainHeight optimizer')
    obj.nX = obj.plant.getNumStates();
    obj.nU = obj.plant.getNumInputs();
    obj.hybridPlant = hybridPlant;
  end


  % adds variables for another trajectory
  function obj = addTrajectoryDecisionVariables(obj, num_knot_points, hybridPlant)
    num_new_state_vars = obj.nX*num_knot_points;
    num_new_control_vars = obj.nU*num_knot_points;

    num_new_time_vars = num_knot_points - 1;
    [obj, new_variable_idx] = obj.addDecisionVariable(num_new_time_vars);
    new_time_variable_idx = new_variable_idx;

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

  % this only applies to the nominal trajecotry
  function obj = addUConstAcrossTransitionsConstraint(obj)
    if options.u_const_across_transitions
      uind = [obj.u_inds(1,1); obj.u_inds(1,end)];
      Q = options.deltaUCostWeight*ones(2,2);
      obj = obj.addCost(QuadraticConstraint(0,inf,Q,b), uind);
    end
  end

  % if traj_idx = 0 then it is the nominal trajectory
  function obj = addGuardConstraints(obj, traj_idx)
    if (traj_idx == 0)
      xinds = obj.x_inds;
      uinds = obj.u_inds;
      hybridPlant = obj.hybridPlant;
      N = obj.N;
    else
      xinds = obj.xtraj_inds{traj_idx};
      uinds = obj.utraj_inds{traj_idx};
      hybridPlant = obj.hybridPlant_array{idx};
      N = obj.N_j{traj_idx};
    end

    % they should all use the default hybrid plant
    hybridPlant = obj.hybridPlant;
    guard_fun = @(x) obj.guardFunction(hybridPlant, x);
    for i=1:N
      guard_lb = 0;
      guard_ub = Inf;

      % guard should be >= 0 except for last knot point where it should be equal to zero
      if (i==N)
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
    end_idx = 2+obj.nX-1;
    df = dg(:,start_idx:end_idx);
  end

end


end
