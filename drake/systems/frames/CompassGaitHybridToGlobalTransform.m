classdef CompassGaitHybridToGlobalTransform < CoordinateTransform
  methods
    function obj=CompassGaitHybridToGlobalTransform(from,to)
      typecheck(from,'CoordinateFrame');
      typecheck(to,'CoordinateFrame');

      feedthroughflag = true;
      tiflag = true;
      obj = obj@CoordinateTransform(from,to,feedthroughflag,tiflag)
      obj=setInputFrame(obj,from);
      obj=setOutputFrame(obj,to);
    end
    
    function ytraj = trajectoryOutput(obj,xtraj,utraj)
      error('if it makes sense for your transform system, implement this to allow trajectories to push through (even if your system is time-varying)');
    end

    function xm = output(obj,~,~,x)
      xm = x;

      % switch them if right leg is swing, i.e. mode 2
      if (xm(1) == 2)
        % right leg is stance, so we need to switch x(2) and x(3) and x(4) and x(5)
        % also need to reverse the sign of u which is x(6)
        xm(2) = x(3);
        xm(3) = x(2);

        xm(4) = x(5);
        xm(5) = x(4);

        xm(6) = -x(6);
      end

    end

  end

  methods(Static)
    function xm = transformLocalToGlobal(obj,~,~,x)
      xm = x;

      % switch them if right leg is swing, i.e. mode 2
      if (xm(1) == 2)
        % right leg is stance, so we need to switch x(2) and x(3) and x(4) and x(5)
        % also need to reverse the sign of u which is x(6)
        xm(2) = x(3);
        xm(3) = x(2);

        xm(4) = x(5);
        xm(5) = x(4);

        xm(6) = -x(6);
      end
    end

    function xm = transformGlobalToLocal(obj,~,~,x)
      xm = x;

      % switch them if right leg is swing, i.e. mode 2
      if (xm(1) == 2)
        % right leg is stance, so we need to switch x(2) and x(3) and x(4) and x(5)
        % also need to reverse the sign of u which is x(6)
        xm(2) = x(3);
        xm(3) = x(2);

        xm(4) = x(5);
        xm(5) = x(4);

        xm(6) = -x(6);
      end
    end

  end


end
