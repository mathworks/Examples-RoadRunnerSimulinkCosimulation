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
    end
end