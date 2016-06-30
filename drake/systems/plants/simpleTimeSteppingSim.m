% just repeatedly calls update on the TSRBM
function [xtraj,output] = simpleTimeSteppingSim(obj,x0,N,controller)
  output = struct(); % struct for putting arbitary garbage into
  nq = getNumPositions(obj);
  nv = getNumVelocities(obj);
  q = x0(1:nq);
  v = x0(nq+1:end);
  assert(numel(v)==nv);

  dt = obj.timestep;
  xValues = repmat(double(x0),1,N+1);
  u = zeros(getNumInputs(obj),1);
  uValues = repmat(u,1,N+1);

  % simulation loop
  for i=1:N
    
    x = xValues(:,i); % the current state
    % query the controller if one was passed in
    if nargin > 3
      u = controller.output(dt*i,[],x);
    end

    xNext = obj.update(dt*i,x,u);
    xValues(:,i+1) = xNext;
  end

  % construct some trajectories for passing back
  xtraj = PPTrajectory(pchip(dt*[0:N], xValues));
  xtraj = xtraj.setOutputFrame(obj.getStateFrame());

  utraj = PPTrajectory(pchip(dt*[0:N], uValues));
  output.utraj = utraj;
end