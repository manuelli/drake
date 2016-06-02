function traj = misim(obj,x0,h,N)
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

% variable indices
vn_inds = 1:nv;
zn_inds = vn_inds(end)+(1:nc);
zf_inds = zn_inds(end)+(1:nc*nd);
lambda_inds = zf_inds(end)+(1:nc);
num_vars = lambda_inds(end);

% constraint indices
dynamic_inds = 1:nv;
no_normal_force_ub_inds = dynamic_inds(end)+(1:nc);
no_tangent_force_lb_inds = no_normal_force_ub_inds(end)+(1:nc);
no_tangent_force_ub_inds = no_tangent_force_lb_inds(end)+(1:nc);
nonpen_lb_inds = no_tangent_force_ub_inds(end)+(1:nc);
nonpen_ub_inds = nonpen_lb_inds(end)+(1:nc);
nonslide_lb_inds = nonpen_ub_inds(end)+(1:nc*nd);
nonslide_ub_inds = nonslide_lb_inds(end)+(1:nc*nd);
num_constraints = nonslide_ub_inds(end);

bigM = 1e2;

% setup model
model.vtype(lambda_inds) = 'B';
model.vtype([vn_inds,zn_inds,zf_inds]) = 'C';
model.obj = zeros(num_vars,1);
model.A = sparse(num_constraints,num_vars);
model.rhs = zeros(num_constraints,1);
model.sense(dynamic_inds)='=';
model.sense([no_tangent_force_lb_inds,nonpen_lb_inds,nonslide_lb_inds])='>';
model.sense([no_normal_force_ub_inds,no_tangent_force_ub_inds,nonpen_ub_inds,nonslide_ub_inds])='<';

model.lb = -inf(num_vars,1);  model.lb(zn_inds)=0;
model.ub = inf(num_vars,1);

model.A(no_normal_force_ub_inds,zn_inds) = eye(nc);
model.A(no_normal_force_ub_inds,lambda_inds) = -eye(nc)*bigM;
% model.rhs(no_normal_force_ub) = 0 already

model.A(no_tangent_force_lb_inds,zf_inds) = eye(nc*nd);
model.A(no_tangent_force_lb_inds,lambda_inds) = eye(nc*nd)*bigM;
% rhs = 0 already
model.A(no_tangent_force_ub_inds,zf_inds) = eye(nc*nd);
model.A(no_tangent_force_ub_inds,lambda_inds) = -eye(nc*nd)*bigM;
% rhs = 0 already

model.A(nonpen_ub_inds,lambda_inds) = eye(nc)*bigM;

model.A(nonslide_lb_inds,lambda_inds) = -eye(nc*nd)*bigM;  % ah.  this is wrong!  there aren't nc*nd lambdas!
model.rhs(nonslide_lb_inds) = -bigM;
model.A(nonslide_ub_inds,lambda_inds) = eye(nc*nd)*bigM;
model.rhs(nonslide_ub_inds) = bigM;

params.outputflag = 0;  % silent output

xx = repmat(double(x0),1,N+1);

for i=1:N
  [H,C] = manipulatorDynamics(obj,q,v);
  [phi,~,~,~,~,~,~,~,n,D] = obj.contactConstraints(q,false);
  d = cell2mat(D(1:nd));
  
  model.A(dynamic_inds,vn_inds) = H/h;
  model.A(dynamic_inds,zn_inds) = -n';
  model.A(dynamic_inds,zf_inds) = -d';
  model.rhs(dynamic_inds) = H*v/h - C; 

  model.A(nonpen_lb_inds,vn_inds) = h*n;
  model.rhs(nonpen_lb_inds) = -phi;
  model.A(nonpen_ub_inds,vn_inds) = h*n;
  model.rhs(nonpen_ub_inds) = bigM - phi;
  
  model.A(nonslide_lb_inds,vn_inds) = d;
  model.A(nonslide_ub_inds,vn_inds) = d;
  
  result = gurobi(model,params);
%  [q',v']
%  lambda = result.x(lambda_inds)'
%  zn = result.x(zn_inds)'
%  zf = result.x(zf_inds)'
  v = result.x(vn_inds);
  q = q+h*v;
  
  xx(:,i+1)=[q;v];
%  keyboard
end

traj = DTTrajectory(h*(0:N),xx);
traj = setOutputFrame(traj,getStateFrame(obj));
