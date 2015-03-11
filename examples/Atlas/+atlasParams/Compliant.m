classdef Compliant < atlasParams.Base
  methods
    function obj = Compliant(r)
      typecheck(r, 'Atlas');
      obj = obj@atlasParams.Base(r);
      upper_body_joints = false(r.getNumPositions,1);
      for i = 1:r.getNumBodies
        if(~isempty(strfind(r.getBody(i).linkname,'arm')) || ~isempty(strfind(r.getBody(i).linkname,'torso')))
          upper_body_joints(r.getBody(i).position_num) = true;
        end
      end
      obj.whole_body.w_qdd(upper_body_joints) = 0.1;
      obj = obj.updateKd();
    end
  end
end


