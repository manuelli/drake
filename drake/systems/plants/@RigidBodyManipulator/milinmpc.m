function [u0,xtraj] = milinmpc(obj,qstar,x0,h,N,Q,R,mu)
% quick demo of mixed-integer-based simulation through contact
% todo: include control inputs (current sets u=0)
% todo: work this into a proper time-stepping system
% todo: add friction cone (currently assumes infinite friction)

nq = getNumPositions(obj);
nv = getNumVelocities(obj);
nu = getNumInputs(obj);
assert(numel(qstar)==nq);
q = x0(1:nq);
v = x0(nq+1:end);
assert(numel(v)==nv);

% compute linearization of constraints
[H,C,B,~,dC] = manipulatorDynamics(obj,qstar,zeros(nv,1));
[phi,~,~,~,~,~,~,~,n,D,dn,dD] = obj.contactConstraints(qstar,false);
nc = numel(phi);
nd = numel(D)/2;
d = cell2mat(D(1:nd));

% compute zn0 (assume zf0 = 0 for now) with C = n'*zn0.
zn0 = zeros(nc,1);  bzn0 = phi<1e-5;
zn0(bzn0) = n(bzn0,:)'\C;

Aq = dC(:,1:nq);
for ci=find(bzn0)'
  Aq = Aq - dn(nq*(ci-1)+(1:nq),:)*zn0(ci);
end
Av = -H/h + dC(:,nq+(1:nv));
Avn = H/h;

if (nargin<8), mu = 1; end

% variable indices
q_inds = reshape(1:nq*N,nv,N);
v_inds = q_inds(end)+reshape(1:nv*(N+1),nv,N+1);
u_inds = v_inds(end)+reshape(1:nu*N,nu,N);
normal_force_inds = reshape(1:nc*N,nc,N);
if (nu>0) normal_force_inds = normal_force_inds+u_inds(end); 
else normal_force_inds = normal_force_inds+v_inds(end);
end
friction_force_inds = normal_force_inds(end)+reshape(1:nc*nd*N,nc*nd,N);
binary_normal_inds = friction_force_inds(end)+reshape(1:nc*N,nc,N);
binary_pos_slide_inds = binary_normal_inds(end)+reshape(1:nc*nd*N,nc*nd,N);
binary_neg_slide_inds = binary_pos_slide_inds(end)+reshape(1:nc*nd*N,nc*nd,N);
num_vars = binary_neg_slide_inds(end);

% constraint indices
dynamic_inds = reshape(1:nv*N,nv,N);
so_dynamic_inds = dynamic_inds(end)+reshape(1:nq*N,nq,N);
no_normal_force_ub_inds = so_dynamic_inds(end)+reshape(1:nc*N,nc,N);
nonpen_lb_inds = no_normal_force_ub_inds(end)+reshape(1:nc*N,nc,N);
nonpen_ub_inds = nonpen_lb_inds(end)+reshape(1:nc*N,nc,N);
nonslide_lb_inds = nonpen_ub_inds(end)+reshape(1:nc*nd*N,nc*nd,N);
nonslide_ub_inds = nonslide_lb_inds(end)+reshape(1:nc*nd*N,nc*nd,N);
pos_slide_force_lb_inds = nonslide_ub_inds(end)+reshape(1:nc*nd*N,nc*nd,N);
pos_slide_force_ub_inds = pos_slide_force_lb_inds(end)+reshape(1:nc*nd*N,nc*nd,N);
neg_slide_force_lb_inds = pos_slide_force_ub_inds(end)+reshape(1:nc*nd*N,nc*nd,N);
neg_slide_force_ub_inds = neg_slide_force_lb_inds(end)+reshape(1:nc*nd*N,nc*nd,N);
num_constraints = neg_slide_force_ub_inds(end);

bigM = 5e2;
params.outputflag = 0;  % silent output
xx = repmat(double(x0),1,N+1);

% setup model
model.vtype = repmat('C',num_vars,1);
model.vtype([binary_normal_inds(:),binary_pos_slide_inds(:),binary_neg_slide_inds(:)]) = 'B';
model.obj = zeros(num_vars,1);
model.A = sparse(num_constraints,num_vars);
model.rhs = zeros(num_constraints,1);
model.sense=repmat('=',num_constraints,1);
model.sense([nonpen_lb_inds(:),nonslide_lb_inds(:),pos_slide_force_lb_inds(:),neg_slide_force_lb_inds(:)])='>';
model.sense([no_normal_force_ub_inds(:),nonpen_ub_inds(:),nonslide_ub_inds(:),pos_slide_force_ub_inds(:),neg_slide_force_ub_inds(:)])='<';

model.lb = -inf(num_vars,1);  model.lb(normal_force_inds(:))=0;
model.ub = inf(num_vars,1);

% set initial conditions
model.lb(q_inds(:,1)) = q;  model.ub(q_inds(:,1)) = q;
model.lb(v_inds(:,1)) = v;  model.ub(v_inds(:,1)) = v;

for i=1:N

  %% dynamic constraints
  model.A(dynamic_inds(:,i),q_inds(:,i)) = Aq;
  model.A(dynamic_inds(:,i),v_inds(:,i)) = Av;
  model.A(dynamic_inds(:,i),v_inds(:,i+1)) = H/h;
  model.A(dynamic_inds(:,i),u_inds(:,i)) = -B;
  model.A(dynamic_inds(:,i),normal_force_inds(:,i)) = -n';
  model.A(dynamic_inds(:,i),friction_force_inds(:,i)) = -d';
  model.rhs(dynamic_inds(:,i)) = -C + Aq*qstar; 
  
  %% so constraints: qn = q+h*vn;
  if (i<N)
    model.A(so_dynamic_inds(:,i),q_inds(:,i+1)) = -eye(nq);
    model.A(so_dynamic_inds(:,i),q_inds(:,i)) = eye(nq);
    model.A(so_dynamic_inds(:,i),v_inds(:,i+1)) = h*eye(nv);
    % rhs = 0 
  end 
  
  %% normal force <= binary_normal*bigM
  model.A(no_normal_force_ub_inds(:,i),normal_force_inds(:,i)) = eye(nc);
  model.A(no_normal_force_ub_inds(:,i),binary_normal_inds(:,i)) = -eye(nc)*bigM;
  % rhs = 0 already

  %% 0 <= phi + h*n*vn <= binary_normal*bigM
  %% linearized to phi0 + n*(q-qstar) + h*n*vn
  model.A(nonpen_lb_inds(:,i),q_inds(:,i)) = n;
  model.A(nonpen_lb_inds(:,i),v_inds(:,i+1)) = h*n;
  model.rhs(nonpen_lb_inds(:,i)) = -phi + n*qstar;
  model.A(nonpen_ub_inds(:,i),q_inds(:,i)) = n;
  model.A(nonpen_ub_inds(:,i),v_inds(:,i+1)) = h*n;
  model.A(nonpen_ub_inds(:,i),binary_normal_inds(:,i)) = eye(nc)*bigM;
  model.rhs(nonpen_ub_inds(:,i)) = bigM - phi + n*qstar;

  %% -binary_neg_slide*bigM <= dk*vn <= binary_pos_slide*bigM
  model.A(nonslide_lb_inds(:,i),binary_neg_slide_inds(:,i)) = eye(nc*nd)*bigM;
  model.A(nonslide_lb_inds(:,i),v_inds(:,i+1)) = d;
  % rhs = 0 already
  model.A(nonslide_ub_inds(:,i),binary_pos_slide_inds(:,i)) = -eye(nc*nd)*bigM;
  model.A(nonslide_ub_inds(:,i),v_inds(:,i+1)) = d;
  % rhs = 0 already

  %% -mu*zn <= zf <= -mu*zn + (1-binary_pos_slide)*M
  model.A(pos_slide_force_lb_inds(:,i),normal_force_inds(:,i)) = mu*repmat(eye(nc),nd,1);
  model.A(pos_slide_force_lb_inds(:,i),friction_force_inds(:,i)) = eye(nc*nd);
  % rhs = 0 already
  
  model.A(pos_slide_force_ub_inds(:,i),normal_force_inds(:,i)) = mu*repmat(eye(nc),nd,1);
  model.A(pos_slide_force_ub_inds(:,i),friction_force_inds(:,i)) = eye(nc*nd);
  model.A(pos_slide_force_ub_inds(:,i),binary_pos_slide_inds(:,i)) = eye(nc*nd)*bigM;
  model.rhs(pos_slide_force_ub_inds(:,i)) = bigM;
  
  %% mu*zn - (1-binary_neg_slide)*bigM <= zf <= mu*zn
  model.A(neg_slide_force_lb_inds(:,i),normal_force_inds(:,i)) = -mu*repmat(eye(nc),nd,1);
  model.A(neg_slide_force_lb_inds(:,i),friction_force_inds(:,i)) = eye(nc*nd);
  model.A(neg_slide_force_lb_inds(:,i),binary_neg_slide_inds(:,i)) = -eye(nc*nd)*bigM;
  model.rhs(neg_slide_force_lb_inds(:,i)) = -bigM;
  
  model.A(neg_slide_force_ub_inds(:,i),normal_force_inds(:,i)) = -mu*repmat(eye(nc),nd,1);
  model.A(neg_slide_force_ub_inds(:,i),friction_force_inds(:,i)) = eye(nc*nd);
  % rhs = 0 already
  
end
  
  result = gurobi(model,params);
%  [q',v']
%   contact = [result.x(binary_normal_inds)';
%     result.x(binary_pos_slide_inds)';
%     result.x(binary_neg_slide_inds)']
%  zn = result.x(zn_inds)'
%  zf = result.x(zf_inds)'
  q = result.x(q_inds);
  v = result.x(v_inds(:,1:N));
  xx=[q;v];

xtraj = DTTrajectory(h*(0:N-1),xx);
xtraj = setOutputFrame(xtraj,getStateFrame(obj));

u0 = result.x(u_inds(:,1));