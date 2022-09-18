classdef stateEstimator < matlab.System & matlab.system.mixin.Propagates %#codegen
   properties (Access = private)
       
   end
   
   methods (Access = protected)
       function [q_fil, dq_fil, RadioButton] = stepImpl(obj, cassieOutputs)
          q_fil = zeros(20, 1);
          dq_fil = zeros(20, 1);
          % Pass on Radio Channel
          RadioButton = RadioChannelToButton(cassieOutputs.pelvis.radio.channel);
       end
   end
end