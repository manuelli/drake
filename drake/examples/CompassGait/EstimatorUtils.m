classdef EstimatorUtils
  methods (Static)
    function val = allEstimatorsInSameMode(observerArray)
      val = true;
      hybridMode = observerArray{1}.hybridMode_;

      for i=1:length(observerArray)
        if hybridMode ~= observerArray{i}.hybridMode_
          val = false;
          return;
        end
      end
    end

    function particleSet = particleSetFromObserverArray(observerArray)
      particleSet = {};
      cgUtils = CompassGaitUtils();

      for i=1:length(observerArray)
        particleSet{end+1} = observerArray{i}.getObserverStateAsParticle();
      end
    end

  end
end


