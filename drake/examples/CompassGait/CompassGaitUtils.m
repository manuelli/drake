classdef CompassGaitUtils
  
  properties
    swapMatrix;
    swapMatrixSmall;
  end
  

  % If mode = 1 (i.e. right leg in swing) we are already in the global coordinates
  methods
    function obj = CompassGaitUtils()
      temp = [0,1;1,0];
      obj.swapMatrix = blkdiag(temp,temp);
      obj.swapMatrixSmall = temp;
    end

    function otherMode = getOtherHybridMode(obj, hybridMode)
      if (hybridMode == 1)
        otherMode = 2;
      else
        otherMode = 1;
      end
    end

    function vecLocal = transformVectorGlobalToLocal(hybridMode, vecGlobal)

      if(hybridMode == 1)
        vecLocal = vecGlobal;
      else
        vecLocal(1) = vecGlobal(2);
        vecLocal(2) = vecGlobal(1);
        vecLocal(3) = vecGlobal(4);
        vecLocal(4) = vecGlobal(3);
      end

    end

    function uLocal = transformGlobalControlToLocalControl(obj, hybridMode, uGlobal)
      if (hybridMode == 1)
        uLocal = uGlobal;
      else
        uLocal = -uGlobal;
      end
    end

    function uGlobal = transformLocalControlToGlobalControl(obj, hybridMode, uLocal)
      if (hybridMode == 1)
        uGlobal = uLocal;
      else
        uGlobal = -uLocal;
      end
    end

    function x_local = transformGlobalStateToLocalState(obj, hybridMode, x_global)
      if (hybridMode == 1)
        % left leg swing
        x_local = [x_global.qL; x_global.qR; x_global.vL; x_global.vR];
      else
        % hybridMode = 2, so we are in right leg swing
        x_local = [x_global.qR; x_global.qL; x_global.vR; x_global.vL];
      end
    end

    % x_local = [q_swing, q_stance, v_swing, v_stance]
    % x_global has fields qL, qR, vL, vR
    % hybridMode 1 is left leg swing
    function x_global = transformLocalStateToGlobalState(obj, hyrbidMode, x_local)
      x_global = struct();

      if (hyrbidMode == 1)
        x_global.qL = x_local(1);
        x_global.qR = x_local(2);
        x_global.vL = x_local(3);
        x_global.vR = x_local(4);
      else
        x_global.qL = x_local(2);
        x_global.qR = x_local(1);
        x_global.vL = x_local(4);
        x_global.vR = x_local(3);
      end
    end

    % transform it into some global coordinates
    function [x_global, returnData] = transformStateToGlobal(obj, x)
      x_global = x;

      if(length(x) == 4)
        x_global = obj.swapMatrix*x;
      end

      % this means it includes the mode
      % only transform if we are in mode 2
      if (length(x) == 5 && (x(1) > 1.5))
        x_global(2:end) = obj.swapMatrix*x(2:end);
      end

      if (nargout > 1)
        returnData = struct();
        returnData.right.x = x_global;
        returnData.right.x(1) = 1; % right is mode 1
        returnData.right.q = x_global(2:3);
        returnData.right.v = x_global(4:5);


        x_left = x_global;
        x_left(1) = 2; % left is mode 2
        x_left(2:end) = obj.swapMatrix*x_global(2:end);
        returnData.left.x = x_left;
        returnData.left.q = x_left(2:3);
        returnData.left.v = x_left(4:5);
      end
    end

    function y_global = transformOutputState(obj, y)
      y_global = y;
      y_global(1:end-1) = obj.transformState(y(1:end-1));

      % this means we are in mode 2, so we need to 
      % swap the sign of the control input
      if(y_global(1) > 1.5)
        y_global(end) = -y(end);
      end

    end

    function xOther = transformStateToOtherMode(obj, x)
      xOther = 0*x;
      xOther(1) = 3-x(1);

      xOther(2:end)  = obj.swapMatrix*x(2:end);
    end


    function u_global = transformControlInput(obj, modeVal, u)
      u_global = u;

      if (modeVal > 1.5)
        u_global = -u;
      end
    end


    % transforms dynamics to global coordinates
    function [H_global, C_global, B_global] = transformDynamics(obj, x, H, C, B)
      H_global = H;
      C_global = C;
      B_global = B;

      assert(length(x) == 5); % needs to include the mode info

      % only need to do something if we are in mode 2
      % mode 1 is the default mode
      if (x(1) < 1.5)
        return;
      end

      H_global = H*obj.swapMatrixSmall';
      B_global = -B;

    end
  end

  methods(Static)
    function fell = doesTrajectoryFall(xtraj)
      fell = false;

      tBreaks = xtraj.getBreaks();
      xGrid = xtraj.eval(tBreaks);

      stanceAngle = xGrid(3,:);

      if max(abs(stanceAngle)) > pi/2
        fell = true;
      end
    end
  end

end