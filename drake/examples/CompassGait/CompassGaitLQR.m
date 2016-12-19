classdef CompassGaitLQR < handle

  properties
    plant;
    stancePlant;
    cgUtils_;
    xtraj;
    utraj;
    xtrajPhase;
    utrajPhase;
    ttrajPhase;
    Q;
    R;
    R_inv;
    tol = 1e-3;
    matrixNorm; % just used for storing debugging info
    tGrid;
    S_traj;
    K_traj;
    STraj;
    KTraj;
  end


  methods
    function obj = CompassGaitLQR(plant, xtraj, utraj)
      obj.plant = plant;
      obj.cgUtils_ = CompassGaitUtils();
      obj.stancePlant = plant.modes{1};
      obj.xtraj = xtraj;
      obj.utraj = utraj;
      obj.matrixNorm = [];
      obj.generatePhaseTraj();
    end

    function generatePhaseTraj(obj)
      tBreaks = obj.xtraj.getBreaks();
      xGrid = obj.xtraj.eval(tBreaks);
      phaseGrid = xGrid(2,:);
      uGrid = obj.utraj.eval(tBreaks);

      obj.xtrajPhase = PPTrajectory(pchip(phaseGrid, xGrid));
      obj.utrajPhase = PPTrajectory(pchip(phaseGrid, uGrid));
      obj.ttrajPhase = PPTrajectory(pchip(phaseGrid, tBreaks));
    end


    function dxpdxm = getResetMapJacobian(obj, x)
      mode = 0;
      t = 0;
      u = 0;
      [xp, mode, status, dxp] = obj.plant.collisionDynamics(mode, t, x, u);

      dxpdxm = dxp(:,3:6); % should be 4 x 4
    end


    function [tGrid,S_traj, K_traj] = computeLQRValueFunction(obj,Q,R)

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
        [tGrid,S_traj] = obj.solveRicattiEquationBackwards(S_final, tspan);        

        S_initial = S_traj(end,:,:); % remember we solve backwards in time
        S_initial = reshape(S_initial,4,4);

        S_final_next = resetMapJacobian' * S_initial * resetMapJacobian;

        % print this out on each iteration to monitor convergence
        matrixNormTemp = norm(S_final- S_final_next);
        
        obj.matrixNorm(end+1) = matrixNormTemp;

        if( matrixNormTemp < obj.tol)
          break;
        end

        S_final = S_final_next;
      end

      K_traj = obj.getKTraj(tGrid, S_traj);
      % solve for gain matrix as well
      obj.tGrid = tGrid;
      obj.S_traj = S_traj;
      obj.K_traj = K_traj;


      obj.KTraj = PPTrajectory(pchip(tGrid, K_traj'));
      numTimes = length(tGrid);
      S_grid = zeros(4,4,numTimes);
      for i=1:length(tGrid)
        S_grid(:,:,i) = S_traj(i,:,:);
      end
      
      obj.STraj = PPTrajectory(pchip(tGrid, S_grid));

    end

    function [tGrid, S_traj] = solveRicattiEquationBackwards(obj, S_final, tspan)
      t_final = tspan(2);
      % we need to solve backwards in time, so need to be careful here
      % change of variables to r = t_final - t. Then tspan is the same
      % but we put a -1 in front of our dxdt because we have reversed direction
      % of integration
      odefun = @(t,S_vec) -obj.ricattiDifferentialEquation(t_final - t,S_vec);
      [t, S_vec_traj] = ode45(odefun, tspan, S_final(:));
      S_traj = reshape(S_vec_traj, length(t),4,4);
      tGrid = t_final - t; % remember to pass reversed time
    end


    function dSdt_vec = ricattiDifferentialEquation(obj,t,S_vec)

      S = reshape(S_vec,4,4); % convert vector to a matrix
      x = obj.xtraj.eval(t);
      u = obj.utraj.eval(t); 
      [A,B] = obj.stancePlant.linearize(t,x,u);
      dSdt = -(A'*S + S*A + obj.Q - S*B*obj.R_inv * B'*S); % we are solving forwards in time

      % now we should put dSdt into vector form,
      dSdt_vec = dSdt(:);
    end

    function K_traj = getKTraj(obj, tGrid, S_traj)
      numTimes = length(tGrid);
      K_traj = zeros(numTimes, 4);
      for i=1:length(tGrid)
        t = tGrid(i);
        x = obj.xtraj.eval(t);
        u = obj.utraj.eval(t);
        S = S_traj(i,:,:);
        S = reshape(S,4,4);
        [A,B] = obj.stancePlant.linearize(t,x,u);
        K = obj.R_inv*B'*S;
        K_traj(i,:) = K;
      end
    end

    function [u, data] = getControlInput(obj, t_plan, xLocal)
      data = struct();
      phaseVar = xLocal(2);
      % t_plan = obj.ttrajPhase.eval(phaseVar);

      K = obj.KTraj.eval(t_plan);
      K = reshape(K,1,4);
      x_plan = obj.xtraj.eval(t_plan);
      u_plan = obj.utraj.eval(t_plan);
      u_fb = -K*(xLocal - x_plan);
      u = u_fb + u_plan;
      data.u_plan = u_plan;
      data.u_fb = u_fb;
      data.u = u;
    end




    function [uGlobal, data] = getControlInputFromGlobalState(obj, t_plan, hybridMode, xGlobal)
      xLocal = obj.cgUtils_.transformGlobalStateToLocalState(hybridMode, xGlobal);
      [uLocal, dataLocal] = obj.getControlInput(t_plan, xLocal);
      data = struct();
      uGlobal = obj.cgUtils_.transformLocalControlToGlobalControl(hybridMode, dataLocal.u);
      data.u_fb = obj.cgUtils_.transformLocalControlToGlobalControl(hybridMode, dataLocal.u_fb);
      data.u_plan = obj.cgUtils_.transformLocalControlToGlobalControl(hybridMode, dataLocal.u_plan);

      data.u = uGlobal;
    end

    function [data] = getInfoFromGlobalState(obj, t_plan, hybridMode, xGlobal)
      data = struct();
      data.xLocal = obj.cgUtils_.transformGlobalStateToLocalState(hybridMode, xGlobal);
      phaseVar = data.xLocal(2);
      data.phaseVar = phaseVar;
      % data.xPlan = obj.xtrajPhase.eval(t_plan);
      % data.uPlan = obj.utrajPhase.eval(t_plan);

      data.phaseVar = phaseVar;
      data.xPlan = obj.xtraj.eval(t_plan);
      data.uPlan = obj.utraj.eval(t_plan);
      data.xBar = data.xLocal - data.xPlan;
      data.V = data.xBar'*obj.STraj.eval(t_plan)*data.xBar;
    end

    function V = getValueFunctionFromGlobalState(obj, hybridMode, xGlobal)
      [phaseVar, xLocal, x_plan] = obj.getInfoFromGlobalState(hybridMode, xGlobal);
    end

  end

end