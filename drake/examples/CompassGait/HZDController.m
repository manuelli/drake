classdef HZDController < SimpleController
  
  properties
    compassGaitStancePlant;
    xtrajPhase;
    utrajPhase;
    xdotTrajPhase;
    theta1_idx = 2;
    theta1dot_idx = 4;
    stanceIdx = 3;
    theta2dot_idx = 5;
    Kp = 10;
    Kd = 6;

    hdTraj;
    hdTraj_deriv;
    hdTraj_dderiv;

    xtraj; % the nominal trajectory that is passed in
  end
  
  methods
    function obj = HZDController(compassGaitPlant)
      obj = obj@SimpleController(compassGaitPlant);
      obj.compassGaitStancePlant = compassGaitPlant.modes{1};
      obj.initializeDataHandle();
    end

    function obj = initializeDataHandle(obj)
      obj.dataHandle.data.times = [];
      obj.dataHandle.data.u_ff = [];
      obj.dataHandle.data.u_fb = [];
      obj.dataHandle.data.z = [];
      obj.dataHandle.data.controlDataCells = {};
    end

    function obj = setNominalTrajectory(obj, xtraj)
      obj.xtraj = xtraj;
      obj = obj.setupController();
    end

    function obj = setupController(obj)
      numTrajectories = length(obj.xtraj.traj);
      h_trajs = {};
      h_deriv_trajs = {};
      h_deriv2_trajs = {};

      for i=1:numTrajectories
        traj = obj.xtraj.traj{i}.trajs{2}; % remove the mode
        [h,hDeriv,hDeriv2] = obj.compute_H_Trajectory(traj);
        h_trajs{i} = h;
        h_deriv_trajs{i} = hDeriv;
        h_deriv2_trajs{i} = hDeriv2;
      end
      
      % ensure that the trajectories are in the same frames
      % otherwise HybridTrajectory will complain
      for i=1:numTrajectories
        h_trajs{i} = h_trajs{i}.setOutputFrame(h_trajs{1}.getOutputFrame);
        h_deriv_trajs{i} = h_deriv_trajs{i}.setOutputFrame(h_deriv_trajs{1}.getOutputFrame);
        h_deriv2_trajs{i} = h_deriv2_trajs{i}.setOutputFrame(h_deriv2_trajs{1}.getOutputFrame);
      end

      obj.hdTraj = HybridTrajectory(h_trajs);
      obj.hdTraj_deriv = HybridTrajectory(h_deriv_trajs);
      obj.hdTraj_dderiv = HybridTrajectory(h_deriv2_trajs);
    end

    % x is a trajectory here, shouldn't include the mode
    % just has (theta_sw, theta_st, theta_sw_dot, theta_st_dot)
    function [h, hDeriv, hDeriv2] = compute_H_Trajectory(obj, x)
      tBreaks = x.getBreaks();
      xGrid = x.eval(tBreaks);
      xDeriv = x.fnder(1);
      xDeriv2 = x.fnder(2);

      xTrueGrid = x.eval(tBreaks);
      xDerivTrueGrid = xDeriv.eval(tBreaks);

      hDerivTrueGrid = xTrueGrid(3,:)./xTrueGrid(4,:);

      hDeriv2TrueGrid = xDerivTrueGrid(3,:)./xTrueGrid(4,:).^2 - ...
        hDerivTrueGrid.*(xDerivTrueGrid(4,:))./xTrueGrid(4,:).^2;

      phaseGrid = xGrid(2,:);
      h = PPTrajectory(pchip(phaseGrid, xGrid(1,:)));
      hDeriv = PPTrajectory(pchip(phaseGrid, hDerivTrueGrid));
      hDeriv2 = PPTrajectory(pchip(phaseGrid, hDeriv2TrueGrid));
    end

    % indexed by theta_1 (swing leg angle), rather than time
    function obj = setTrajectory(obj, xtrajPhase, utrajPhase, xdotTrajPhase)
        obj.xtrajPhase = xtrajPhase;
        obj.utrajPhase = utrajPhase;
        obj.xdotTrajPhase = xdotTrajPhase;

        obj.hdTraj = xtrajPhase(obj.theta1_idx);
        obj.hdTraj_deriv = obj.hdTraj.fnder(1);
        obj.hdTraj_dderiv = obj.hdTraj.fnder(2);
    end
    
    function obj = step(obj,t,x,u)
      obj.dataHandle.data.tickCounter = obj.dataHandle.data.tickCounter + 1;
      obj.dataHandle.data.times = [obj.dataHandle.data.times;t];
      [u, controlData] = testGetControlInput(obj,t,x);

      % bookkeeping: store some control data
      obj.dataHandle.data.currentControlInput = u;
      obj.dataHandle.data.u(end+1) = u;
      obj.dataHandle.data.controlDataCells{end+1} = controlData;
    end
    
    function u = getCurrentControlInput(obj)
      u = obj.dataHandle.data.currentControlInput;
    end


    function [u, controlData] = testGetControlInput(obj,t,x, verbose)
      if nargin < 4
        verbose = false;
      end

      controlData = struct();
      % Figure out what plan is telling us to do
      phaseVar = x(obj.stanceIdx);
      controlData.phaseVar = phaseVar;

      % clip the phaseVar to limits
      % could this be messing us up? maybe need to do a linear interpolation
      % off of the ends
      tspan = obj.hdTraj.tspan;
      phaseVar = max(tspan(1), min(tspan(2), phaseVar));

      thetaSwing_des = obj.hdTraj.eval(phaseVar);

      theta1 = x(obj.theta1_idx);
      theta1dot = x(obj.theta1dot_idx);
      theta2dot = x(obj.theta2dot_idx);

      h_deriv = [1, -obj.hdTraj_deriv.eval(phaseVar)];
      h_d_dderiv = obj.hdTraj_dderiv.eval(phaseVar);

      y = theta1 - thetaSwing_des;
      ydot = h_deriv*[theta1dot; theta2dot];


      % desired acceleration of y
      % this is just the PD law
      z = -obj.Kp*y - obj.Kd*ydot; 


      q = x(2:3);
      v = x(4:5);
      [H,C,B] = obj.compassGaitStancePlant.manipulatorDynamics(q,v);

      temp = (h_deriv*inv(H)*B)^(-1);
      uStar = temp*(h_d_dderiv*theta2dot^2 + h_deriv*inv(H)*C);
      u_fb = temp*z;

      u = uStar + u_fb;

      
      controlData.u = u;
      controlData.uStar = uStar;
      controlData.u_fb = u_fb;
      controlData.y = y;
      controlData.ydot = ydot;
      controlData.z = z;
      controlData.phaseVarClipped = phaseVar;
    end


    % function controlTrajs = makeControlDataTrajectories(obj,t,controlDataCells)
    %   % numTimes = length(obj.dataHandle.data.controlDataCells);

    %   % t = obj.dataHandle.data.times;
    %   numTimes = length(t);
    %   uStar = zeros(numTimes);
    %   u_fb = zeros(numTimes);
    %   y = zeros(numTimes);
    %   ydot = zeros(numTimes);
    %   z = zeros(numTimes);
    %   phaseVar = zeros(numTimes);
    %   phaseVarClipped = zeros(numTimes);

    %   for i=1:numTimes
    %     % t(i) = obj.dataHandle.data.controlDataCells{i}.t
    %     uStar(i) = obj.dataHandle.data.controlDataCells{i}.uStar;
    %     u_fb(i) = obj.dataHandle.data.controlDataCells{i}.u_fb;
    %     y(i) = obj.dataHandle.data.controlDataCells{i}.y;
    %     ydot(i) = obj.dataHandle.data.controlDataCells{i}.ydot;
    %     z(i) = obj.dataHandle.data.controlDataCells{i}.z;
    %     phaseVar(i) = obj.dataHandle.data.controlDataCells{i}.phaseVar;
    %     phaseVarClipped(i) = obj.dataHandle.data.controlDataCells{i}.phaseVarClipped;
    %   end

    %   controlTrajs = struct();
    %   % controlTrajs.traj.t = PPTrajectory(pchip(t, t));
    %   controlTrajs.traj.uStar = PPTrajectory(pchip(t, uStar));
    %   controlTrajs.traj.u_fb = PPTrajectory(pchip(t, u_fb));
    %   controlTrajs.traj.y = PPTrajectory(pchip(t, y));
    %   controlTrajs.traj.ydot = PPTrajectory(pchip(t, ydot));
    %   controlTrajs.traj.z = PPTrajectory(pchip(t, z));
    %   controlTrajs.traj.phaseVar = PPTrajectory(pchip(t, phaseVar));
    %   controlTrajs.traj.phaseVarClipped = PPTrajectory(pchip(t, phaseVarClipped));

    %   controlTrajs.grid.t = t;
    %   controlTrajs.grid.uStar = uStar;
    %   controlTrajs.grid.u_fb = u_fb;
    %   controlTrajs.grid.y = y;
    %   controlTrajs.grid.ydot = ydot;
    %   controlTrajs.grid.z = z;
    %   controlTrajs.grid.phaseVar = phaseVar;
    %   controlTrajs.grid.phaseVarClipped = phaseVarClipped;
    % end

    function controlTrajs = makeControlDataTrajectories(obj,t,controlDataCells)
      % numTimes = length(obj.dataHandle.data.controlDataCells);

      % t = obj.dataHandle.data.times;
      numTimes = length(t);
      u = zeros(numTimes, 1);
      uStar = zeros(numTimes,1);
      u_fb = zeros(numTimes,1);
      y = zeros(numTimes,1);
      ydot = zeros(numTimes,1);
      z = zeros(numTimes,1);
      phaseVar = zeros(numTimes,1);
      phaseVarClipped = zeros(numTimes,1);

      for i=1:numTimes
        % t(i) = obj.dataHandle.data.controlDataCells{i}.t
        u(i) = controlDataCells{i}.u;
        uStar(i) = controlDataCells{i}.uStar;
        u_fb(i) = controlDataCells{i}.u_fb;
        y(i) = controlDataCells{i}.y;
        ydot(i) = controlDataCells{i}.ydot;
        z(i) = controlDataCells{i}.z;
        phaseVar(i) = controlDataCells{i}.phaseVar;
        phaseVarClipped(i) = controlDataCells{i}.phaseVarClipped;
      end

      controlTrajs = struct();
      % controlTrajs.traj.t = PPTrajectory(pchip(t, t));
      controlTrajs.traj.u = PPTrajectory(pchip(t,u));
      controlTrajs.traj.uStar = PPTrajectory(pchip(t, uStar));
      controlTrajs.traj.u_fb = PPTrajectory(pchip(t, u_fb));
      controlTrajs.traj.y = PPTrajectory(pchip(t, y));
      controlTrajs.traj.ydot = PPTrajectory(pchip(t, ydot));
      controlTrajs.traj.z = PPTrajectory(pchip(t, z));
      controlTrajs.traj.phaseVar = PPTrajectory(pchip(t, phaseVar));
      controlTrajs.traj.phaseVarClipped = PPTrajectory(pchip(t, phaseVarClipped));

      controlTrajs.grid.t = t;
      controlTrajs.grid.u = u;
      controlTrajs.grid.uStar = uStar;
      controlTrajs.grid.u_fb = u_fb;
      controlTrajs.grid.y = y;
      controlTrajs.grid.ydot = ydot;
      controlTrajs.grid.z = z;
      controlTrajs.grid.phaseVar = phaseVar;
      controlTrajs.grid.phaseVarClipped = phaseVarClipped;
    end


    function returnData = reconstructControlDataFromTrajectory(obj, xtraj)
      tBreaks = xtraj.getBreaks();
      numTimes = length(tBreaks);
      controlDataCells = {};

      for i=1:numTimes
        t = tBreaks(i);
        [u,controlData] = obj.testGetControlInput(t, xtraj.eval(t));
        controlDataCells{end+1} = controlData;
      end

      returnData = obj.makeControlDataTrajectories(tBreaks, controlDataCells);
    end


    % % theta1ddot is the desired acceleration of theta1
    % function u_fl = computeFeedbackLinearization(obj,t,x_full,theta1ddot)
    %   x = x_full(2:end);
    %   q = x(1:2);
    %   v = x(3:4);
    %   [H,C,B] = obj.compassGaitStancePlant.manipulatorDynamics(q,v);
    %   H_inv = inv(H);
    %   a = -H_inv*C;
    %   a = a(1);

    %   b = H_inv*B;
    %   b = b(1);

    %   % then we have 
    %   % z = theta_1_ddot = a + bu
      
    %   u_fl = (theta1ddot - a)/b;
    % end

  end
  
end