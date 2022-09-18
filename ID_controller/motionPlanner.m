classdef motionPlanner < matlab.System & matlab.system.mixin.Propagates %#codegen
    properties (Access = private)
        
    end
    
    methods (Access = protected)
        function[ddq_desired] = stepImpl(obj, q_fil, dq_fil, RadioButton)
           ddq_desired = q_fil + dq_fil; 
        end
    end
end