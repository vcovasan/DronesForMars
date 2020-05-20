close all

% Flags plot - put 1 to enable and 0 to disable
flag_plot.all      = 0;
flag_plot.plant    = 0;
flag_plot.lat      = 0;
flag_plot.lqr      = 0;
flag.lqg_eval = 0;
flag_plot.compare_lqr_lqg = 0;


% % Constant definitions
% g0    = 32.17405; %gravity (feet/s2)
% vt    = 21.9543;  %total air speed velocity (free-stream)
% theta_trim = 0;   %pitch trim angle at wings-level
% 
% %Stability derivatives
% Ybeta = -4.5129;
% Yp    = -0.05579;
% Yail  = 0;
% Yr    = 0;
% Lp    = -0.3295;
% Lr    = 0.0205;
% Lail  = 3.6299;
% Lbeta = 3.7096;
% Nail  = 3.0316;
% Np    = 0.02025;
% Nr    = -0.10266;
% Nbeta = 0.79937;

%Defining the state
flag_beta_state = 0;
state_name  = {'v','p','r','phi'};
output_name = {'v_s','p_s','r_s','phi_s'};

E_lat = eye(4);
if flag_beta_state == 1
    E_lat  = diag([vt 1 1 1]);
    state_name = {'beta','p','r','phi'};
    output_name = {'beta_s','p_s','r_s','phi_s'};
end

%Analytical model from the book: AIRCRAFT CONTROL AND SIMULATION
%Ap_lat = [Ybeta Yp Yr-vt g0*cos(theta_trim);    Lbeta Lp Lr 0;Nbeta Np Nr 0;0 1 tan(theta_trim) 0];

%Ap_lat = E_lat\Ap_lat;

%From the article the matrices is:
%Ap_lat = [-0.2051 -0.05579 -21.9543 32.174;   -0.1686 -0.3295 0.0205 0;0.03633 0.02025 -.10266 0; 0 1 0 0];
Ap_lat = [-0.000625746 0.00200502 -126.31 3.711;-0.149438 -1.74874 0.172113 0; 0.00783305 -0.085404 -0.001846 0; 0 1 0 0]

% Bp_lat = [0 Lail Nail 0]';

Bp_lat = [-0.00370476 36.8612 -0.05977335 0]'

Cp_lat = eye(4);

Dp_lat = zeros(4,1);

%Build the linear model in state space
Gp_lat = ss(Ap_lat,Bp_lat,Cp_lat,Dp_lat,'inputname','ail','statename', state_name,...
    'outputname',output_name);

% Actuator dynamics
% Create the actuator dynamics. I put this dynamics to show how the
% actuator dynamics will impact the stability margins of the system.

wn_act   = 5*2*pi; %rad/s
zeta_act = 0.7;
At_ss = ss([0 1; -wn_act^2 -2*zeta_act*wn_act],[0; wn_act^2], [1 0], 0,'statename',{'act_pos','act_vel'},'inputname','ail_cmd','outputname','ail');

%Dynamics of plant augmented with the actuator dynamics
Gp_lat_aug = connect(Gp_lat,At_ss,'ail_cmd',output_name);

%Plot the frequency response and pzmap of the bare-airframe
if flag_plot.all == 1 || flag_plot.plant == 1
    figure()
    bode(Gp_lat,Gp_lat_aug)
    legend('Bare Airframe','Augmented Plant')
    figure()
    pzmap(Gp_lat,Gp_lat_aug)
    legend('Bare Airframe','Augmented Plant')
end

%-------------------- Design the LQG Controller --------------------------%

%% First step - LQR Design - J = x'Qx + u'Ru
% Here I am desiging a roll-tracking controller

%Augment the plant of an integrator of roll error in the feedforward path
%of the controller
Caux         = Cp_lat(4,:);
[nr, nc ] = size(Ap_lat);
A_aug     = [Ap_lat, zeros(nr,1); -Caux, 0];
B_aug     = [Bp_lat;              0 ];

% Second step - tuning the controller based on Bryson's  Rule.
q_beta = 1/5^2;
q_p = 1/1^2;
q_phi = 1/0.5^2;
q_r = 1/5^2;
q_int_phi = 2*q_phi;

% Defining the Q weight and R
Q = diag([q_beta q_p  q_r q_phi q_int_phi]);
R = 1/1^2;

Klqr_lat_gain = lqr(A_aug, B_aug, Q , R);

K_v       = Klqr_lat_gain(1); %proportional gain
K_p       = Klqr_lat_gain(2);
K_r       = Klqr_lat_gain(3);
K_phi     = Klqr_lat_gain(4);
K_int_phi = -Klqr_lat_gain(5);

% controller at state space
Kc_lat    = ss(0,[0 0 0 -1 1], K_int_phi, [-K_v -K_p -K_r -K_phi 0], 'statename','err_phi', 'inputname', {'v_s','p_s', 'r_s','phi_s', 'phi_cmd'}, 'outputname','ail');
Kc_lat_lo = ss(0,[0 0 -1 1], K_int_phi, [-K_p -K_r -K_phi 0],      'statename','err_phi', 'inputname', {'p_s', 'r_s','phi_s', 'phi_cmd'}, 'outputname','ail');

% 1 - option to calculate the openloop and closed-loop by definition
%Open-loop stability Margin from full feedback
Ga_lat_lqr = ss(A_aug, B_aug, Klqr_lat_gain, 0);

% Closed_loop - for full feedback
Gf_lat_lqr = ss(A_aug  - B_aug*Klqr_lat_gain, [zeros(nr,1); 1 ], eye(nr+1), zeros(nr+1,1));

%2 - Using matlab commands as I will show below
Gf_lat_lqr_aux = connect(Gp_lat,Kc_lat,'phi_cmd', output_name,'ail');
Ga_lat_lqr_aux = -getIOTransfer(Gf_lat_lqr_aux,'ail','ail','ail');


Gf_lat_lqr_lo = connect(Gp_lat,Kc_lat_lo,'phi_cmd', output_name,'ail');
Ga_lat_lqr_lo = -getIOTransfer(Gf_lat_lqr_lo,'ail','ail','ail');

%Showing the results and prove the both options given the same results
if flag_plot.lqr == 1 || flag_plot.all ==1
    figure()
    step(Gf_lat_lqr,Gf_lat_lqr,Gf_lat_lqr_lo)
    legend('Option 1', 'Option 2', 'Removing the v state')
    grid on
    figure()
    bode(Ga_lat_lqr,Ga_lat_lqr_aux,Ga_lat_lqr_lo)
    legend('Option 1', 'Option 2', 'Removing the v state')
    grid on
end

%% Second step - Build the LQG
% I choose to synthesize using the LGQ/LTR at plant input.

rho = [0.01 1 10 1000 ];                %tuning knob
rho = [0.01 1 10];                %tuning knob
Q0  = 0.001*eye(4);                     %process  noise
Rn  = blkdiag(0.01, 0.001,0.0001);     %measure  noise

Clo_lat    = [0 1 0 0; 0 0 1 0; 0 0 0 1]; %C matrix given the available matrix

aux_str    = cell(numel(rho)+1,1);
aux_str{1} = 'LQR';

for i=1:numel(rho)
    
    aux_str{i+1} = ['rho = ' num2str(rho(i))];
    
    Qf = Q0 + 1/rho(i)*(Bp_lat*Bp_lat');           %Given by LTR at plant Input
    Klqg_lat = lqe(Ap_lat,eye(4),Clo_lat, Qf, Rn); %Instead you can use lqe command
    
    %Build the LQE controller at state space format
    Aest = Ap_lat - Klqg_lat*Clo_lat;
    Best = [Bp_lat Klqg_lat];
    Cest = eye(4);
    Dest = zeros(4,4);
    Klqe_lat = ss(Aest,Best,Cest,Dest,'statename',{'v_e','p_e','r_e','phi_e'},'inputname',{'ail','p_s','r_s', 'phi_s'},'outputname',{'v_hat','p_hat','r_hat','phi_hat'});
    
    %Build the full controller + Optimal Regulator + Optimal Estimator
    Klqr_lat = Kc_lat;
    Klqr_lat.InputName = {'v_hat','p_hat','r_hat','phi_hat','phi_cmd'};
    Gf_lat_lqg = connect(Gp_lat,Klqr_lat,Klqe_lat,'phi_cmd',{'v_s','p_s','r_s','phi_s'},'ail');
    Ga_lat_lqg = -getIOTransfer(Gf_lat_lqg,'ail','ail','ail');
    
    %Here we have the same answer due to perfect matching of plant dynamics
    %in LQG design.
    if flag_plot.all == 1 || flag_plot.compare_lqr_lqg == 1 && i==1
        figure()
        bode(Ga_lat_lqr_aux,Ga_lat_lqg)
        legend('LQR','LQG')
        figure()
        step(Gf_lat_lqr_aux,Gf_lat_lqg)
        legend('LQR','LQG')
    end
    
    % Including the actuator dynamics
    
    
    %lqr
    Klqr_aux = Kc_lat;
    Klqr_aux.OutputName = 'ail_cmd';
    
    Gf_lat_lqr_aug = connect(Klqr_aux, Gp_lat_aug,'phi_cmd',{'phi_s','p_s','r_s','v_s'},'ail_cmd');
    Ga_lat_lqr_aug = -getIOTransfer(Gf_lat_lqr_aug,'ail_cmd','ail_cmd','ail_cmd');
    
    %lqg
    Klqr_aux = Kc_lat;
    Klqr_aux.InputName  = {'v_hat','p_hat','r_hat','phi_hat','phi_cmd'};
    Klqr_aux.OutputName = 'ail_cmd';
    Klqe_lat.InputName{1}  = 'ail_cmd';
    Gf_lat_lqg_aug = connect(Klqr_aux, Klqe_lat, Gp_lat_aug,'phi_cmd',{'phi_s','p_s','r_s','v_s'},'ail_cmd');
    Ga_lat_lqg_aug = -getIOTransfer(Gf_lat_lqg_aug,'ail_cmd','ail_cmd','ail_cmd');
    
    
    %     Klqr_aux.OutputName = 'ail_cmd_Out';
    %     Ga_teste = -connect(Klqr_aux, Klqe_lat, Gp_lat_aug,'ail_cmd','ail_cmd_Out');
    
    if i==1 && flag.lqg_eval == 1
        h = gcf;
        figure()
        bode(Ga_lat_lqr_aug)
        grid on
        figure()
        step(Gf_lat_lqr_aug)
        grid on
        figure()
        nyquist(Ga_lat_lqr_aug)
        grid on
    end
    
    if flag.lqg_eval == 1
        figure(h.Number +1)
        hold on
        bode(Ga_lat_lqg_aug)
        figure(h.Number +2)
        hold on
        step(Gf_lat_lqg_aug)
        figure(h.Number +3)
        hold on
        nyquist(Ga_lat_lqg_aug)
    end
    
end

if flag.lqg_eval == 1
    %Insert the legends
    figure(h.Number+1)
    legend(aux_str)
    
    figure(h.Number+2)
    legend(aux_str)
    
    figure(h.Number+3)
    legend(aux_str)
end
