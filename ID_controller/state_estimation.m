function [state] = state_estimation(obj_memory, cassieOutputs, time, MODE, ground_truth, isSim)

state = CassieState;

% If Cassie not calibrated, state estimation return default values.
if ~cassieOutputs.isCalibrated
    return;
end

% memory from previous iteration (obj is the memory object)
stanceLeg = obj_memory.stanceLeg;
dq_fil_prev = obj_memory.dq_fil;
p_StSw_I0 = obj_memory.p_StSw_I0;
p_BSw_I0 = obj_memory.p_BSw_I0;
q_spring_fil_prev = obj_memory.q_spring_fil;
dq_spring_fil_prev = obj_memory.dq_spring_fil;

% Define parameters.
stance_thres_lb = 100;
stance_thres_ub = 200;
s_max = 1.05;
grf_threshold = 50; % 50;

%% Get phase variable.
% completion of the current gait.
% s<0.5 is rising, s>0.5 is going down
if MODE == 0
    time.last_contact_time = time.t;
end
s = (time.t-time.last_contact_time)/time.step_time;

%% Get joint states.
[qyaw, qpitch, qroll, dqyaw, dqpitch, dqroll] = IMU_to_Euler_v2(cassieOutputs.pelvis.vectorNav.orientation, cassieOutputs.pelvis.vectorNav.angularVelocity);

qa = CassieModule.getDriveProperty(cassieOutputs,'position');
if (isSim == 0)
   qa(2) = qa(2) - cassieOutputs.pelvis.radio.channel(5) * 10;
end
dqa = CassieModule.getDriveProperty(cassieOutputs,'velocity');
qj = CassieModule.getJointProperty(cassieOutputs,'position');
dqj = CassieModule.getJointProperty(cassieOutputs,'velocity');
qq = cassieOutputs.pelvis.vectorNav.orientation;

R_WB = Rot(qyaw,'z');
R_BP = Rot([qpitch; qroll],'yx'); % R_BP = roty(rad2deg(qpitch))*rotx(rad2deg(qroll));
R_WP = R_WB * R_BP;

% Raw robot frame acceleration measured from IMU frame.
a_P = cassieOutputs.pelvis.vectorNav.linearAcceleration; 
a_W = R_WP * a_P;
a_W = a_W - [0;0;9.806]; % World frame acceleration without gravity.
% if IRC.reset_bias % Put z_bias into memory.
%     obj.z_bias = YToolkits.first_order_filter(obj.z_bias, a_W(3), 0.0002);
% end
% a_W(3) = a_W(3) - obj.z_bias;  %(in world coordinate, z_bias is not gravity, but compensation for other factors)

qaL = qa(1:5);
qaR = qa(6:10);
qjL = qj(1:2);
qjR = qj(4:5);

% Get current velocities
dqaL = dqa(1:5);
dqaR = dqa(6:10);
dqjL = dqj(1:2);
dqjR = dqj(4:5);

qall = [zeros(3,1); qyaw; qpitch; qroll; ...
    qaL(1:4); qjL; qaL(5);...
    qaR(1:4); qjR; qaR(5)];

dqall = [zeros(3,1); dqyaw; dqpitch; dqroll; ...
    dqaL(1:4); dqjL; dqaL(5);...
    dqaR(1:4); dqjR; dqaR(5)];

% Create states in pelvis B frame by making pelvis x,y,z,yaw zero.
qall_B = qall; qall_B(1:4) = 0;
dqall_B = dqall; dqall_B(1:4) = 0;

% Create states in pelvis P frame by making all pelvis states zero.
qall_P = qall; qall_P(1:6) = 0;
dqall_P = dqall; dqall_P(1:6) = 0;

%% Get and Filter spring constants (also for GRF estimation)
state.qall = qall; state.dqall = dqall; % Use these half-done data temporarily For using functions in cassie state.
q_spring = state.get_spring_deflection();
dq_spring = state.get_spring_deflection_rate();

% Filter to avoid unstable control caused by spring oscillation
if MODE == 0 % standing requires high filtering
    para_q_spring = 0.003;
else
    para_q_spring = 0.05;
end
q_spring_fil = (1-para_q_spring)*q_spring_fil_prev + para_q_spring*q_spring;

% dt = 5e-4; % 0.5 ms simulation rate 2000Hz
% dq_spring_fil = (q_spring_fil - q_spring_fil_prev)/dt;
para_dq_spring = 0; % filtered data is zero
dq_spring_fil = (1-para_dq_spring)*dq_spring_fil_prev + para_dq_spring*dq_spring;

%% Get contact state.
% get GRF and regulized stance.
if cassieOutputs.isCalibrated == 1
    [ GRF_L, GRF_R ] = get_GRF(qall,q_spring);
else
    GRF_L = [0;0];
    GRF_R = [0;0];
end
GRF_v = [GRF_L(2); GRF_R(2)];

% s is normalized between 0 and 1, 0 means the leg is in air and 1 means leg is on ground.
stance_L = (GRF_v(1)-stance_thres_lb)/(stance_thres_ub-stance_thres_lb);
stance_R = (GRF_v(2)-stance_thres_lb)/(stance_thres_ub-stance_thres_lb);
stance_L = median([0,1,stance_L]);
stance_R = median([0,1,stance_R]);
stance_LR = [stance_L; stance_R];

% Distribute GRF.
if stanceLeg == 1 % right leg stance/in contact.
    swing_grf = GRF_L(2);
    stance_grf = GRF_R(2);
else
    swing_grf = GRF_R(2);
    stance_grf = GRF_L(2);
end

%% Detection foot switch.
foot_strike = s > 0.9 && swing_grf > grf_threshold; % determine foot strike by ground reaction forces and if swing foot is going down (to prevent multiple switches in one strike).
gait_cycle_end = s > s_max;

foot_switch_happens = MODE==1 && (foot_strike || gait_cycle_end); % must in walking
if foot_switch_happens
    stanceLeg = -stanceLeg;
    s = 0;
    time.last_contact_time = time.t;
end

% Get toe bottom positions in pelvis B frame.
[p_BTl, p_BTr] = get_feet_position(qall_B); % FK in this function is defined in pelvis B or P frame
p_ITl = p_BTl + [3.155; 0; -7.996]/100; %p_LeftToeJoint(qall_B);
p_ITr = p_BTr + [3.155; 0; -7.996]/100; %p_RightToeJoint(qall_B);
if stanceLeg == 1 % RightStance
  p_BSt = p_BTr;  
  p_BSw = p_BTl;
  p_ISt = p_ITr;
  p_ISw = p_ITl;
else % -1, LeftStance
  p_BSt = p_BTl;
  p_BSw = p_BTr;
  p_ISt = p_ITl;
  p_ISw = p_ITr;
end
p_StSw = p_BSw - p_BSt;

if foot_switch_happens || MODE==0
    p_StSw_I0 = p_StSw;
    p_BSw_I0 = p_BSw;
else
    % account for state estimation error. z_StSw should be 0 when double stance.
    p_StSw_I0(3) = 0.99*p_StSw_I0(3) + 0.01*0; % force z-direction to 0 over time
end

%% Get Pelvis state. Core part of state estimation.
% Dependent on stanceLeg.
dq = get_velocity_v3(qall_B,dqall_B,stanceLeg);
pelvis_state = pelvis_kalman_filter(obj_memory, qyaw, p_ISt, foot_switch_happens, a_W, s, time);

dqall(1:3) = dq; % pelvis_state.dq;
% Filter the velocity.
para = [0.003;0.0211;0.03]; %changed by William (1st-order cheby1 filter)
dq_fil = dq_fil_prev.*(1-para) + dq.*para; % first-order filtering (smoothing) (use previous value)
% Use ground truth to replace dq_fil (for now)
dq_W_gt = ground_truth(8:10);
dq_B_gt = R_WB' * dq_W_gt;

%% Estimate pelvis state in Middle frame. This works for standing, and not meaningful for walking.
p_MB = zeros(3,1);
p_MB(1:2) = -1/2*(p_BTl(1:2)+p_BTr(1:2)); % Middle frame is in the middle of two feet, aliged with Toe frame.
p_MB(3) = -p_BSt(3) + state.h_foot;
qall(1:3) = p_MB;

%% Get center of mass data (Added by William, from FG controller)
p_BC = ComPosition(qall_B)';
v_BC = ComVelocity(qall_B,dqall_B);

%% Put in.
state.qall = qall;
state.dqall = dqall;
state.dq_fil = dq_B_gt; %dq_fil;

state.t = time.t;
state.last_contact_time = time.last_contact_time;
state.step_time = time.step_time;
state.s = s;

state.stanceLeg = stanceLeg;
state.stance_LR = stance_LR;
state.GRF_v = GRF_v;

% William added
state.foot_switch_happens = foot_switch_happens;
state.p_BTr = p_BTr;
state.p_BTl = p_BTl;
state.p_StSw = p_StSw;
state.p_StSw_I0 = p_StSw_I0;
state.p_BSw_I0 = p_BSw_I0;
state.p_BSw = p_BSw;
state.p_BSt = p_BSt;

state.p_BC = p_BC;
state.v_BC = v_BC;

% pelvis kalman filter memory.
state.x_IT_kf = pelvis_state.x_IT_kf;
state.y_IT_kf = pelvis_state.y_IT_kf;
state.z_IT_kf = pelvis_state.z_IT_kf;
state.Sigma_x_IT = pelvis_state.Sigma_x_IT;
state.Sigma_y_IT = pelvis_state.Sigma_y_IT;
state.Sigma_z_IT = pelvis_state.Sigma_z_IT;

state.q_spring_fil = q_spring_fil;
state.dq_spring_fil = dq_spring_fil;

end
