classdef MIStep < handle

  properties
    plant;
    model;
    params;
    options;
    idx;
    nq;
    nv;
    nc;
    nd;
    dt;
    bigM;
  end

  methods

    function obj = MIStep(plant, dt, options)
      if nargin < 3
        options = struct()
      end

      obj.options = applyDefaults(options,struct('mu',1))
      obj.plant = plant;
      obj.nq = plant.getNumPositions();
      obj.nv = plant.getNumVelocities();
      obj.dt = dt;
      obj.setupModel();
    end

    function obj = setupModel(obj)
      nq = obj.nq;
      nv = obj.nv;
      dt = obj.dt;
      q = zeros(nq,1);
      v = zeros(nv,1);

      [phiC,~,~,~,~,~,~,~,n,D] = obj.plant.contactConstraints(q,false);
      nc = numel(phiC);
      nd = numel(D)/2;

      obj.nc = nc;
      obj.nd = nd;

      obj.idx = struct();

      % variable indices
      vn_inds = 1:nv;
      normal_force_inds = vn_inds(end)+(1:nc);
      friction_force_inds = normal_force_inds(end)+(1:nc*nd);
      binary_normal_inds = friction_force_inds(end)+(1:nc);
      binary_pos_slide_inds = binary_normal_inds(end)+(1:nc*nd);
      binary_neg_slide_inds = binary_pos_slide_inds(end)+(1:nc*nd);
      num_vars = binary_neg_slide_inds(end);

      % constraint indices
      dynamic_inds = 1:nv;
      no_normal_force_ub_inds = dynamic_inds(end)+(1:nc);
      nonpen_lb_inds = no_normal_force_ub_inds(end)+(1:nc);
      nonpen_ub_inds = nonpen_lb_inds(end)+(1:nc);
      nonslide_lb_inds = nonpen_ub_inds(end)+(1:nc*nd);
      nonslide_ub_inds = nonslide_lb_inds(end)+(1:nc*nd);
      pos_slide_force_lb_inds = nonslide_ub_inds(end)+(1:nc*nd);
      pos_slide_force_ub_inds = pos_slide_force_lb_inds(end)+(1:nc*nd);
      neg_slide_force_lb_inds = pos_slide_force_ub_inds(end)+(1:nc*nd);
      neg_slide_force_ub_inds = neg_slide_force_lb_inds(end)+(1:nc*nd);
      num_constraints = neg_slide_force_ub_inds(end);



      % store these in the object properties
      obj.idx.vn_inds = vn_inds
      obj.idx.normal_force_inds = normal_force_inds;
      obj.idx.friction_force_inds = friction_force_inds;
      obj.idx.binary_normal_inds = binary_normal_inds;
      obj.idx.binary_pos_slide_inds = binary_pos_slide_inds;
      obj.idx.binary_neg_slide_inds = binary_neg_slide_inds;
      obj.idx.num_vars = num_vars;

      % constraint indices
      obj.idx.dynamic_inds = dynamic_inds;
      obj.idx.no_normal_force_ub_inds = no_normal_force_ub_inds;
      obj.idx.nonpen_lb_inds = nonpen_lb_inds;
      obj.idx.nonpen_ub_inds = nonpen_ub_inds;
      obj.idx.nonslide_lb_inds = nonslide_lb_inds;
      obj.idx.nonslide_ub_inds = nonslide_ub_inds;
      obj.idx.pos_slide_force_lb_inds = pos_slide_force_lb_inds;
      obj.idx.pos_slide_force_ub_inds = pos_slide_force_ub_inds;
      obj.idx.neg_slide_force_lb_inds = neg_slide_force_lb_inds;
      obj.idx.neg_slide_force_ub_inds = neg_slide_force_ub_inds;
      obj.idx.num_constraints = num_constraints;

      % construct the model and params structs
      bigM = (0.02/dt)*(1e3);
      obj.bigM = bigM
      % setup model
      model.vtype = repmat('C',num_vars,1);
      model.vtype([binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds]) = 'B';
      model.obj = zeros(num_vars,1);
      model.A = sparse(num_constraints,num_vars);
      model.rhs = zeros(num_constraints,1);
      model.sense(dynamic_inds)='=';
      model.sense([nonpen_lb_inds,nonslide_lb_inds,pos_slide_force_lb_inds,neg_slide_force_lb_inds])='>';
      model.sense([no_normal_force_ub_inds,nonpen_ub_inds,nonslide_ub_inds,pos_slide_force_ub_inds,neg_slide_force_ub_inds])='<';

      model.lb = -inf(num_vars,1);  model.lb(normal_force_inds)=0;
      model.ub = inf(num_vars,1);

      %% normal force <= binary_normal*bigM
      model.A(no_normal_force_ub_inds,normal_force_inds) = eye(nc);
      model.A(no_normal_force_ub_inds,binary_normal_inds) = -eye(nc)*bigM;
      % rhs = 0 already


      %% 0 <= phi + h*n*vn <= (1-binary_normal)*bigM
      % The binary variables should only be active when phi is 0 or negative?
      model.A(nonpen_ub_inds,binary_normal_inds) = eye(nc)*bigM;



      %% -binary_neg_slide*bigM <= dk*vn <= binary_pos_slide*bigM 

      % this has something to do with non-sliding constraint
      model.A(nonslide_lb_inds,binary_neg_slide_inds) = eye(nc*nd)*bigM;  
      % rhs = 0 already
      model.A(nonslide_ub_inds,binary_pos_slide_inds) = -eye(nc*nd)*bigM;
      % rhs = 0 already

      %% -mu*zn <= zf <= -mu*zn + (1-binary_pos_slide)*M
      model.A(pos_slide_force_lb_inds,normal_force_inds) = obj.options.mu*repmat(eye(nc),nd,1);
      model.A(pos_slide_force_lb_inds,friction_force_inds) = eye(nc*nd);
      % rhs = 0 already

      model.A(pos_slide_force_ub_inds,normal_force_inds) = obj.options.mu*repmat(eye(nc),nd,1);
      model.A(pos_slide_force_ub_inds,friction_force_inds) = eye(nc*nd);
      model.A(pos_slide_force_ub_inds,binary_pos_slide_inds) = eye(nc*nd)*bigM;
      model.rhs(pos_slide_force_ub_inds) = bigM;

      %% mu*zn - (1-binary_neg_slide)*bigM <= zf <= mu*zn
      model.A(neg_slide_force_lb_inds,normal_force_inds) = -obj.options.mu*repmat(eye(nc),nd,1);
      model.A(neg_slide_force_lb_inds,friction_force_inds) = eye(nc*nd);
      model.A(neg_slide_force_lb_inds,binary_neg_slide_inds) = -eye(nc*nd)*bigM;
      model.rhs(neg_slide_force_lb_inds) = -bigM;

      model.A(neg_slide_force_ub_inds,normal_force_inds) = -obj.options.mu*repmat(eye(nc),nd,1);
      model.A(neg_slide_force_ub_inds,friction_force_inds) = eye(nc*nd);

      % store in the object properties
      obj.model = model;
      obj.params.outputflag = 0;
    end


    function [xn, result] = step(obj,t,x,u)
      q = x(1:obj.nq);
      v = x(obj.nq+1:end);

      model = obj.model;
      params = obj.params;

      [H,C,B] = manipulatorDynamics(obj.plant,q,v);
      [phi,~,~,~,~,~,~,~,n,D] = obj.plant.contactConstraints(q,false);
      d = cell2mat(D(1:obj.nd));
      
      model.A(obj.idx.dynamic_inds,obj.idx.vn_inds) = H/obj.dt;
      model.A(obj.idx.dynamic_inds,obj.idx.normal_force_inds) = -n';
      model.A(obj.idx.dynamic_inds,obj.idx.friction_force_inds) = -d';
      model.rhs(obj.idx.dynamic_inds) = H*v/obj.dt - C + B*u; 

      model.A(obj.idx.nonpen_lb_inds,obj.idx.vn_inds) = obj.dt*n;
      model.rhs(obj.idx.nonpen_lb_inds) = -phi;
      model.A(obj.idx.nonpen_ub_inds,obj.idx.vn_inds) = obj.dt*n;
      model.rhs(obj.idx.nonpen_ub_inds) = obj.bigM - phi;
      
      model.A(obj.idx.nonslide_lb_inds,obj.idx.vn_inds) = d;
      model.A(obj.idx.nonslide_ub_inds,obj.idx.vn_inds) = d;
      
    %   % for debugging
    %   bnorm = (phi<.1);
    %   bpos = zeros(nc*nd,1);
    %   bneg = zeros(nc*nd,1);
    %   model2 = model;
    %   model2.rhs = -model.A(:,[binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds])*[bnorm;bpos;bneg];
    %   model2.A(:,[binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds])=[];
    %   model2.vtype([binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds])=[];
    %   model2.obj([binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds])=[];
    %   model2.lb([binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds])=[];
    %   model2.ub([binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds])=[];
    %   % end debugging
      
      
      result = gurobi(model,params);

      % this keeps being empty, which I assume means an infeasible solve . . . ?
      result.status;
      if ~strcmp(result.status, 'OPTIMAL')
        disp('didnt get and OPTIMAL solution from gurobi')
      end

    %  [q',v']
    %   contact = [result.x(binary_normal_inds)';
    %     result.x(binary_pos_slide_inds)';
    %     result.x(binary_neg_slide_inds)']
    %  zn = result.x(zn_inds)'
    %  zf = result.x(zf_inds)'
      vn = result.x(obj.idx.vn_inds);
      qn = q+obj.dt*vn;
      
      xn=[qn;vn];
    end
  end

end