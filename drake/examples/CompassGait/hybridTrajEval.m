function y = hybridTrajEval(traj, t)
  numTimes = length(t);
  for i=1:numTimes
    y(:,i) = traj.eval(t(i));
  end
end