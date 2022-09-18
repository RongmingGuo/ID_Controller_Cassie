classdef signalWrapper < matlab.System & matlab.system.mixin.Propagates
   properties(Access = private)
       
   end
   
   methods(Access = protected)
       function [controlInputs] = stepImpl(obj, cassieOutputs, u)
           controlInputs = 0;
       end
   end
end