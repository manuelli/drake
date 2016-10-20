classdef CompassGaitLQR

  properties
    plant;
    stancePlant;
    xtraj;
    utraj;
    Q;
    R;
    R_inv;
    tol = 1e-3;
    matrixNorm; % just used for storing debugging info
  end


  methods
    function obj = CompassGaitLQR(plant, xtraj, utraj)
      obj.plant = plant;
      obj.stancePlant = plant.modes{1};
      obj.xtraj = xtraj;
      obj.utraj = utraj;
      obj.matrixNorm = [];
    end


    function dxpdxm = getResetMapJacobian(obj, x)
      mode = 0;
      t = 0;
      u = 0;
      [xp, mode, status, dxp] = obj.plant.collisionDynamics(mode, t, x, u);

      dxpdxm = dxp(:,3:6); % should be 4 x 4
    end


    function [tGrid,S_traj] = computeLQRValueFunction(obj,Q,R)

      if nargin < 3
        Q = eye(4);
        R = 0.1;
      end

      S_final = eye(4);
      tspan = obj.xtraj.tspan;
      t_final = tspan(2);
      t_initial = tspan(1);

      obj.Q = Q;
      obj.R = R;
      obj.R_inv = inv(R);

      resetMapJacobian = obj.getResetMapJacobian(obj.xtraj.eval(t_final));

      % run the while loop
      while(true)
        [tGrid,S_traj] = obj.solveRicattiEquationForwards(S_final, tspan);        

        S_initial = S_traj(end,:,:); % remember we solve backwards in time
        S_initial = reshape(S_initial,4,4);

        S_final_next = resetMapJacobian' * S_initial * resetMapJacobian;

        % print this out on each iteration to monitor convergence
        matrixNormTemp = norm(S_final- S_final_next)
        
        obj.matrixNorm(end+1) = matrixNormTemp;

        if( matrixNormTemp < obj.tol)
          break;
        end

        S_final = S_final_next;
      end
    end

    function [tGrid, S_traj] = solveRicattiEquationForwards(obj, S_initial, tspan)
      t_final = tspan(2);
      % we need to solve backwards in time, so need to be careful here
      % change of variables to r = t_final - t. Then tspan is the same
      % but we put a -1 in front of our dxdt because we have reversed direction
      % of integration
      odefun = @(t,S_vec) -obj.ricattiDifferentialEquation(t_final - t,S_vec);
      [t, S_vec_traj] = ode45(odefun, tspan, S_initial(:));
      S_traj = reshape(S_vec_traj, length(t),4,4);
      tGrid = t_final - t; % remember to pass reversed time
    end


    function dSdt_vec = ricattiDifferentialEquation(obj,t,S_vec)

      S = reshape(S_vec,4,4); % convert vector to a matrix
      x = obj.xtraj.eval(t);
      [A,B] = obj.stancePlant.linearize(t,x,0);
      dSdt = -(A'*S + S*A + obj.Q - S*B*obj.R_inv * B'*S); % we are solving forwards in time

      % now we should put dSdt into vector form,
      dSdt_vec = dSdt(:);
    end

  end

end