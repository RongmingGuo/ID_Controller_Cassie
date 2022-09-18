classdef inverseDynamics < matlab.System & matlab.system.mixin.Propagates
    properties (Access = private)
        %% Spring Joint Constants
        k_l_shin = 100;
        k_l_tarsus = 100;
        k_r_shin = 100;
        k_r_tarsus = 100;
    end
    
    methods (Access = protected)
        function[userInputs] = stepImpl(obj, q_fil, dq_fil, ddq_desired)
           % Calculate Inertia Matrix
           M = InertialMatrix(q_fil);
           % Calculate Coriolis Term
           H = CoriolisTerm(q_fil, dq_fil);
           % Calculate Gracity Vector
           G = GravityVector(q_fil);
           % Estimate Ground Reaction Force
           torque_GRF = zeros(20, 1);
           % Calculate Passive Torques
           torque_Spring = zeros(20, 1);
           torque_Spring([11, 12, 18, 19]) = q_fil([11, 12, 18, 19]) .* [obj.k_l_shin; obj.k_l_tarsus; obj.k_r_shin; obj.k_r_tarsus];
           % Apply the inverse dynamics equation
           u = M * ddq_desired + H + G - torque_GRF - torque_Spring;
           u = u([7:10, 13, 14:17, 20]);
           
           userInputs.torque = u;
        end
    end
end