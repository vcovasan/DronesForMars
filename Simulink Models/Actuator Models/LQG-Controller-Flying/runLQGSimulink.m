% Script to control the simulink
% Remember to run first the LonAP_FlightWing.m to PitchControl or
% LateraAP_FlightWing.m to Lateral autopilot

%flag to control the simulation
flag_pitch_control = 1; %choose between lon and lat dynamics to simulate

if flag_pitch_control == 1 
    run LonAP_FilghtWing.m 
    sim_name = 'LQG_UAV_LonAp';
else
    run LateralAP_FlightWing.m
    sim_name = 'LQG_UAV_LatAp';
end


flag_all_sim       = 1; %enable simulation of all controller to compare (see the flags below)

flag_controller_aux = [0 0 0 0 1 1 1 1 2 2 2 2]; % 0 - LQG, 1 - LQR - Output Feebback Controller (PID)
flag_act_aux   =      repmat([0 1 0 1],1,3);     % 0 - No actuator dynamics 1 - actuator dynamics
flag_noise_aux = repmat([0 0 1 1],1,3);          % 0 - Noise disable , 1 - Noise enable



%Choose which case do you wanna simulate see the order in the flags above
idx_i = 2;
idx_f = idx_i;

if flag_all_sim == 1
    idx_i = 1;
    idx_f = numel(flag_act_aux);   
end


for i=idx_i:idx_f
    
    fprintf('Simulation number %d %s\n',i,sim_name);
    flag_controller = flag_controller_aux(i);
    flag_act = flag_act_aux(i);
    flag_noise = flag_noise_aux(i);
    
    if flag_pitch_control ==1
        sim('LQG_UAV_LonAP')
    else
        sim('LQG_UAV_LateralAP')
    end
    
    
    
end
