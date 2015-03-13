classdef CompliantBodyMotion < atlasParams.Base
  methods
    function obj = CompliantBodyMotion(r)
      typecheck(r, 'Atlas');
      nbod = r.getManipulator().getNumBodies();
      obj = obj@atlasParams.Base(r);

      % set the body motion weights to be non-zero for all bodies
      obj.body_motion.weight = num2cell(0.1*ones(1,nbod));

      % do we still want w_qdd weights???, for now will leave them the same as in the 
      % standing parameters

      % in simulation all joints are actually force controlled, will need a hardware replica
      % of this class that specifies that all joints be force controlled
      obj = obj.updateKd();
    end
  end
end
