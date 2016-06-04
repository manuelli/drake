function traj = misim(obj,x0,h,N,mu)
% quick demo of mixed-integer-based simulation through contact
% todo: include control inputs (current sets u=0)
% todo: work this into a proper time-stepping system
% todo: add friction cone (currently assumes infinite friction)

nq = getNumPositions(obj);
nv = getNumVelocities(obj);
q = x0(1:nq);
v = x0(nq+1:end);
assert(numel(v)==nv);

[phiC,~,~,~,~,~,~,~,n,D] = obj.contactConstraints(q,false);
nc = numel(phiC);
nd = numel(D)/2;
if (nargin<5), mu = 1; end

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

bigM = 5e2;

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

%% phi + h*n*vn <= binary_normal*bigM
model.A(nonpen_ub_inds,binary_normal_inds) = eye(nc)*bigM;

%% -binary_neg_slide*bigM <= dk*vn <= binary_pos_slide*bigM 
model.A(nonslide_lb_inds,binary_neg_slide_inds) = eye(nc*nd)*bigM;  
% rhs = 0 already
model.A(nonslide_ub_inds,binary_pos_slide_inds) = -eye(nc*nd)*bigM;
% rhs = 0 already

%% -mu*zn <= zf <= -mu*zn + (1-binary_pos_slide)*M
model.A(pos_slide_force_lb_inds,normal_force_inds) = mu*repmat(eye(nc),nd,1);
model.A(pos_slide_force_lb_inds,friction_force_inds) = eye(nc*nd);
% rhs = 0 already

model.A(pos_slide_force_ub_inds,normal_force_inds) = mu*repmat(eye(nc),nd,1);
model.A(pos_slide_force_ub_inds,friction_force_inds) = eye(nc*nd);
model.A(pos_slide_force_ub_inds,binary_pos_slide_inds) = eye(nc*nd)*bigM;
model.rhs(pos_slide_force_ub_inds) = bigM;

%% mu*zn - (1-binary_neg_slide)*bigM <= zf <= mu*zn
model.A(neg_slide_force_lb_inds,normal_force_inds) = -mu*repmat(eye(nc),nd,1);
model.A(neg_slide_force_lb_inds,friction_force_inds) = eye(nc*nd);
model.A(neg_slide_force_lb_inds,binary_neg_slide_inds) = -eye(nc*nd)*bigM;
model.rhs(neg_slide_force_lb_inds) = -bigM;

model.A(neg_slide_force_ub_inds,normal_force_inds) = -mu*repmat(eye(nc),nd,1);
model.A(neg_slide_force_ub_inds,friction_force_inds) = eye(nc*nd);
% rhs = 0 already


contact_ind = 2;
contact_constraint_inds = [nonslide_lb_inds(contact_ind),nonslide_ub_inds(contact_ind),pos_slide_force_lb_inds(contact_ind),pos_slide_force_ub_inds(contact_ind),neg_slide_force_lb_inds(contact_ind),neg_slide_force_ub_inds(contact_ind)];
contact_var_inds = [normal_force_inds(contact_ind),friction_force_inds(contact_ind),binary_normal_inds(contact_ind),binary_pos_slide_inds(contact_ind),binary_neg_slide_inds(contact_ind)];
syms xn yn tn real;
syms zn zf bnorm bpos bneg real;  x=[xn;yn;tn;zn;zf;bnorm;bpos;bneg];


params.outputflag = 1; %0;  % silent output

xx = repmat(double(x0),1,N+1);

for i=1:N
  [H,C] = manipulatorDynamics(obj,q,v);
  [phi,~,~,~,~,~,~,~,n,D] = obj.contactConstraints(q,false);
  d = cell2mat(D(1:nd));
  
  model.A(dynamic_inds,vn_inds) = H/h;
  model.A(dynamic_inds,normal_force_inds) = -n';
  model.A(dynamic_inds,friction_force_inds) = -d';
  model.rhs(dynamic_inds) = H*v/h - C; 

  model.A(nonpen_lb_inds,vn_inds) = h*n;
  model.rhs(nonpen_lb_inds) = -phi;
  model.A(nonpen_ub_inds,vn_inds) = h*n;
  model.rhs(nonpen_ub_inds) = bigM - phi;
  
  model.A(nonslide_lb_inds,vn_inds) = d;
  model.A(nonslide_ub_inds,vn_inds) = d;
  
  % for debugging
  bnorm = (phi<.1);
  bpos = zeros(nc*nd,1);
  bneg = zeros(nc*nd,1);
  model2 = model;
  model2.rhs = -model.A(:,[binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds])*[bnorm;bpos;bneg];
  model2.A(:,[binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds])=[];
  model2.vtype([binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds])=[];
  model2.obj([binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds])=[];
  model2.lb([binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds])=[];
  model2.ub([binary_normal_inds,binary_pos_slide_inds,binary_neg_slide_inds])=[];
  % end debugging
  
  
  result = gurobi(model,params);
%  [q',v']
  contact = [result.x(binary_normal_inds)';
    result.x(binary_pos_slide_inds)';
    result.x(binary_neg_slide_inds)']
%  zn = result.x(zn_inds)'
%  zf = result.x(zf_inds)'
  v = result.x(vn_inds);
  q = q+h*v;
  
  xx(:,i+1)=[q;v];
%  keyboard
end

traj = DTTrajectory(h*(0:N),xx);
traj = setOutputFrame(traj,getStateFrame(obj));
