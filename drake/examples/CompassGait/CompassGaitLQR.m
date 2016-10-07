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
  end


  methods
    function obj = CompassGaitLQR(plant, xtraj, utraj)
      obj.plant = plant;
      obj.stancePlant = plant.modes{1};
      obj.xtraj = xtraj;
      obj.utraj = utraj;
    end


    function dxpdxm = getResetMapJacobian(obj, x)
      mode = 0;
      t = 0;
      u = 0;
      dxp = obj.plant.collisionDynamics(mode, t, x, u);

      dxpdxm = dxp(:,3:6); % should be 4 x 4
    end


    function S_traj = computeLQRValueFunction(obj,Q,R)

      if nargin < 3
        Q = eye(4);
        R = 0.1;
      end

      S_initial = eye(4);
      tspan = obj.xtraj.tspan;
      t_final = tspan(2);
      t_initial = tspan(1);

      obj.Q = Q;
      obj.R = R;
      obj.Rinv = inv(R);

      resetMapJacobian = obj.getResetMapJacobian(obj, xtraj.eval(t_final));

      % run the while loop
      while(true)
        S_traj = obj.solveRicattiEquationForwards(S_initial, tspan);
        S_final = S_traj.eval(t_final);
        S_initial_next = resetMapJacobian'*S_final*resetMapJacobian;

        S_initial = S_traj.eval(t_initial);

        % print this out on each iteration to monitor convergence
        matrixNorm = norm(S_initial - S_initial_next)

        if( matrixNorm < obj.tol)
          break;
        end

        S_initial = S_initial_next;
      end
    end

    function S_traj = solveRicattiEquationForwards(obj, S_initial, tspan)
      odefun = @(t,S) obj.ricattiDifferentialEquation(obj,t,S);
      [t, S_traj] = ode45(odefun, tspan, S_initial);
    end


    function dSdt = ricattiDifferentialEquation(obj,t,S)
      x = xtraj.eval(t);
      [A,B] = obj.stancePlant.linearize(t,x,0);
      dSdt = -(A'*S + S*A' + obj.Q - S*B*obj.Rinv * obj.B'*S);
    end

  end

end