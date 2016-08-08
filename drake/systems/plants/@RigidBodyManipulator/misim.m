function [traj, output] = misim(obj,x0,h,N,mu,controller,viz)
% quick demo of mixed-integer-based simulation through contact
% todo: include control inputs (current sets u=0)
% todo: work this into a proper time-stepping system
% todo: add friction cone (currently assumes infinite friction)


% @param h is the timestep dt
% @param N number of timesteps to simulate
% @mu friction coefficient

output = struct();

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

% make sure we scale this with dt to avoid infeasibility problems
% Impulse delivered at impact step is independent of dt. Since impulse
% is I = F dt this means that F GROWS linearly with dt, so we must ensure that
% we also increase M accordingly.
% Note: h = dt in this notation
bigM = (0.02/h)*(1e3);

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
 
 
% contact_ind = 2;
% contact_constraint_inds = [nonslide_lb_inds(contact_ind),nonslide_ub_inds(contact_ind),pos_slide_force_lb_inds(contact_ind),pos_slide_force_ub_inds(contact_ind),neg_slide_force_lb_inds(contact_ind),neg_slide_force_ub_inds(contact_ind)];
% contact_var_inds = [normal_force_inds(contact_ind),friction_force_inds(contact_ind),binary_normal_inds(contact_ind),binary_pos_slide_inds(contact_ind),binary_neg_slide_inds(contact_ind)];
% syms xn yn tn real;
% syms zn zf bnorm bpos bneg real;  x=[xn;yn;tn;zn;zf;bnorm;bpos;bneg];


params.outputflag = 0;  % silent output

alphaValues = zeros(num_vars,1);
alphaValues = repmat(alphaValues, 1,N+1);
xx = repmat(double(x0),1,N+1);
u = zeros(getNumInputs(obj),1);
uValues = repmat(u,1,N+1);
JvnSingle = zeros(nc,1);
JvnValues = repmat(JvnSingle, 1, N+1);

vSingle = zeros(nv, 1);
JTransposeF = repmat(vSingle, 1, N+1);

for i=1:N
  [H,C,B] = manipulatorDynamics(obj,q,v);
  [phi,~,~,~,~,~,~,~,n,D] = obj.contactConstraints(q,false);
  if (nargin>5) 
    u = controller.output(h*i,[],[q;v]);
  end
  d = cell2mat(D(1:nd));
  
  model.A(dynamic_inds,vn_inds) = H/h;
  model.A(dynamic_inds,normal_force_inds) = -n';
  model.A(dynamic_inds,friction_force_inds) = -d';
  model.rhs(dynamic_inds) = H*v/h - C + B*u; 

  model.A(nonpen_lb_inds,vn_inds) = h*n;
  model.rhs(nonpen_lb_inds) = -phi;
  model.A(nonpen_ub_inds,vn_inds) = h*n;
  model.rhs(nonpen_ub_inds) = bigM - phi;
  
  model.A(nonslide_lb_inds,vn_inds) = d;
  model.A(nonslide_ub_inds,vn_inds) = d;
  
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
    error('the problem is infeasible')
    break;
  end

%  [q',v']
%   contact = [result.x(binary_normal_inds)';
%     result.x(binary_pos_slide_inds)';
%     result.x(binary_neg_slide_inds)']
%  zn = result.x(zn_inds)'
%  zf = result.x(zf_inds)'
  v = result.x(vn_inds);
  q = q+h*v;
  if (nargin>6)
    viz.drawWrapper(i*h,q); 
  end
  
  xx(:,i+1)=[q;v];
  alphaValues(:,i+1) = result.x;
  uValues(:,i+1) = u;
  JvnValues(:,i+1) = n*v; % encodes distance traveled by contact point this tick
  JTransposeF(:,i+1) = n'*result.x(normal_force_inds) + d'*result.x(friction_force_inds);
%  keyboard
end

traj = PPTrajectory(pchip(h*(0:N),xx));
traj = setOutputFrame(traj,getStateFrame(obj));

% record all the debug output
output.alphaTraj = PPTrajectory(pchip(h*(0:N),alphaValues));
output.uTraj = PPTrajectory(pchip(h*(0:N),uValues));
JTransposeFTraj = PPTrajectory(pchip(h*(0:N),JTransposeF));
JvnTraj = PPTrajectory(pchip(h*(0:N),JvnValues));


output.traj = traj;
output.vn_inds = vn_inds;
output.normal_force_inds = normal_force_inds;
output.friction_force_inds = friction_force_inds;
output.binary_normal_inds = binary_normal_inds;
output.binary_pos_slide_inds = binary_pos_slide_inds;
output.binary_neg_slide_inds = binary_neg_slide_inds;
output.num_vars = num_vars;
output.JTransposeFTraj = JTransposeFTraj
output.JvnTraj = JvnTraj
