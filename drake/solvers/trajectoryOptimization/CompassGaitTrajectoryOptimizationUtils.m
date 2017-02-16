classdef CompassGaitTrajectoryOptimizationUtils

methods(Static)
  function [g,dg] = controlInputQuadraticCost(t,x,u)
    R = 1;
    g = sum((R*u).*u,1);
    dg = [zeros(1,1+size(x,1)),2*u'*R];
  end

  function [g,dg] = controlInputQuadraticCostIntegrated(t,x,u)
    R = 1;
    uSquared = sum((R*u).*u,1);
    g = t*uSquared;
    dg = [uSquared, zeros(1,size(x,1)), 2*u'*R*t]; 
  end

end


end
