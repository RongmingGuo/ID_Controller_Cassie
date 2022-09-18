classdef InertiaMatrixBlock < matlab.System & matlab.system.mixin.Propagates
    properties(Access = private)
        
    end
    
    methods(Access = protected)
        function[M] = stepImpl(obj, q_fil)
           M = InertiaMatrix(q_fil);
        end
    end
end