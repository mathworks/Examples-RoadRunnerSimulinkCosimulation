classdef CRG_xy2z < matlab.System
    properties(Nontunable)
        crgData;  % crg_eval_xy2zに渡すデータ
    end
    
    methods (Access = protected)
    function setupImple(obj)
        
    end
        function y = stepImpl(obj, u)
            y = crg_eval_xy2z(obj.crgData, u);
        end

      function out = getOutputSizeImpl(obj)
         out = [4, 1];
      end
    
      function out = isOutputComplexImpl(obj)
         out = propagatedInputComplexity(obj, 1);
      end
    
      function out = getOutputDataTypeImpl(obj)
         out = propagatedInputDataType(obj, 1);
      end
    
      function out = isOutputFixedSizeImpl(obj)
         out = true;
      end
    end

end