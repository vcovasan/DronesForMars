
close all

% Flags plot - put 1 to enable and 0 to disable
flag_plot.all      = 0;
flag_plot.plant    = 0;
flag_plot.pitch    = 0;
flag.lqg_eval = 0;
flag_plot.compare_lqr_lqg = 0;


% Bare-Airframe
%Ap = [-6.68 4.1754 0 -32.174;   -0.6276 -3.0503 19.34 0; -0.01376 0.05852 -0.1179 0; 0 0 1 0];
Ap=[-0.00403548 0.0511006 0 -3.711; -0.155603 -1.04255 126.189 0; 0.000417069 -3.3536 -1.15905 0; 0 0 1 0 ]
% Bp = [-0.649; 26.0063; -1.1526; 0];
%thatOne = (1/0.01189)*(-0.0063095+(-0.081571)*(-0.00564));
%BlongMars = [1.390234 -4.430383 -70.44032 0; 1.1133 0 thatOne 0]'
Bp = [1.390234 -4.430383 -70.44032 0]'

Cp = eye(4);
Dp = zeros(4,1);

%Create the model at state space format
Gp_lon = ss(Ap,Bp,Cp,Dp,'statename',{'u','w','q','theta'},'inputname',{'elev'}, 'outputname',{'u_s','w_s','q_s','theta_s'});

%% Actuator dynamics
% Create the actuator dynamics. I put this dynamics to show how the
% actuator dynamics will impact the stability margins of the system.

wn_act   = 5*2*pi; %rad/s
zeta_act = 0.7;
At_ss = ss([0 1; -wn_act^2 -2*zeta_act*wn_act],[0; wn_act^2], [1 0], 0,'statename',{'act_pos','act_vel'},'inputname','elev_cmd','outputname','elev');

Gp_lon_aug = connect(Gp_lon, At_ss,'elev_cmd',{'u_s','w_s','q_s','theta_s'});

%Plot the frequency response and pzmap of the bare-airframe
if flag_plot.all == 1 || flag_plot.plant == 1
    figure()
    bode(Gp_lon,Gp_lon_aug)
    grid on
    legend('Bare Airframe', 'Augmented')
    figure()
    pzmap(Gp_lon,Gp_lon_aug)
    legend('Bare Airframe', 'Augmented')
end



%-------------------- Design the LQG Controller --------------------------%

%% First step - LQR Design
% Here I am desiging a pitch-tracking controller

%Augment the plant of an integrator of pitch error in the feedforward path
%of the controller
C = Cp(4,:);
[nr, nc ] = size(Ap);
A_aug = [Ap, zeros(nr,1); -C, 0];
B_aug = [Bp;              0 ];

% Second step - tuning the controller based on Bryson rule
%
q_theta = 1/0.1^2;
q_q = 1/5^2;
q_w = 1/5^2;
q_u = 1/5^2;
q_int_erro = 2*q_theta;

Q = diag([q_u q_w q_q q_theta q_int_erro]);
R = 1/1^2;

Klqr_gain = lqr(A_aug,B_aug, Q , R);

K_u         = Klqr_gain(1); %proportional gain
K_w         = Klqr_gain(2);
K_q         = Klqr_gain(3);
K_theta     = Klqr_gain(4);
K_int_theta = -Klqr_gain(5);


%Full feedback all states are feed
%Open-loop stability Margin from full feedback
Ga_lon_lqr = ss(A_aug, B_aug, Klqr_gain, 0);

% Closed_loop - for full feedback
Gf_lon_lqr_aux = ss(A_aug  - B_aug*Klqr_gain, [zeros(nr,1); 1 ], eye(nr+1), zeros(nr+1,1));

%Controller in state space
Kc_pitch = ss(0,[0 0 0 -1 1], K_int_theta, [-K_u -K_w -K_q -K_theta 0], 'statename','err_theta', 'inputname', {'u_s','w_s', 'q_s','theta_s', 'theta_cmd'}, 'outputname','elev');

%reduced controller (elimated the u_s and v_s state variables)
Kc_pitch_lo = ss(0,[1 -1 0], K_int_theta, [0 -K_theta -K_q], 'inputname', {'theta_cmd', 'theta_s','q_s'}, 'outputname','elev');

%Convert
Gf_lon_lqr    = connect(Gp_lon,Kc_pitch,'theta_cmd',{'theta_s','q_s','w_s','u_s'},'elev');
Gf_lon_lqr_lo = connect(Gp_lon,Kc_pitch_lo,'theta_cmd',{'theta_s','q_s','w_s','u_s'},'elev');

%Opee-Loop
Ga_pitch    = -getIOTransfer(Gf_lon_lqr,'elev','elev','elev');
Ga_pitch_lo = -getIOTransfer(Gf_lon_lqr_lo,'elev','elev','elev');

%Plots for pitch-hold controller
%Note the difference between full-state feedback and reduced is smaller
%with the simplified of model of the plant (no noise, actuator dynamics,
%etc)
if flag_plot.all == 1 || flag_plot.pitch == 1
    figure()
    step(Gf_lon_lqr,Gf_lon_lqr_lo,10)
    legend('Full Feedback', 'Removing u and w state')
    grid on
    
    figure()
    pzmap(Gf_lon_lqr,Gf_lon_lqr_lo)
    legend('Full Feedback - LQR', 'Removing u and w state from controller')
    grid on
    
    figure()
    bode(Ga_pitch,Ga_pitch_lo)
    legend('Full Feedback - LQR', 'Removing u and w state  from controller')
    title('OpenLoop Pitch-Hold')
    grid on
end

%% Second step - Design optimal state estimator
% I choose to synthesize using the LGQ/LTR at plant input.

rho = [0.0001 10 100 1000];            %tuning knob
Q0  = 0.001*eye(4);               %process  noise
Rn  = blkdiag(0.001, 0.0001);     %measure  noise

Clo    = [0 0 1 0; 0 0 0 1];

aux_str    = cell(numel(rho)+1,1);
aux_str{1} = 'LQR';

for i=1:numel(rho)
    
    aux_str{i+1} = ['rho = ' num2str(rho(i))];
    
    Qf = Q0 + 1/rho(i)*(Bp*Bp');
    Klqg_test = lqr(Ap',Clo',Qf,Rn)';        %By duality you can use LQR to calculate LQE
    Klqg = lqe(Ap,eye(4),Clo, Qf, Rn); %Instead you can use lqe command
    
    
    Aest = Ap - Klqg*Clo;
    Best = [Bp Klqg];
    Cest = eye(4);
    Dest     = zeros(4,3);
    Klqe_lon = ss(Aest,Best,Cest,Dest,'statename',{'u_e','w_e','q_e','theta_e'},'inputname',{'elev','q_s','theta_s'},'outputname',{'u_hat','w_hat','q_hat','theta_hat'});
    
    %% Build the full controller + Optimal Regulator + Optimal Estimator
    Klqr = Kc_pitch;
    Klqr.InputName = {'u_hat','w_hat','q_hat','theta_hat','theta_cmd'};
    Gf_pitch_lqg = connect(Gp_lon,Klqr,Klqe_lon,'theta_cmd',{'theta_s','q_s','w_s','u_s','theta_hat'},'elev');
    Ga_pitch_lqg = -getIOTransfer(Gf_pitch_lqg,'elev','elev','elev');
    
    
    %Here we have the same answer due to perfect matching of plant dynamics
    %in LQG design.
    if (flag_plot.all == 1 || flag_plot.compare_lqr_lqg == 1) && i==1
        figure()
        bode(Ga_pitch,Ga_pitch_lqg)
        legend('LQR','LQG')
        figure()
        step(Gf_lon_lqr,Gf_pitch_lqg)
        legend('LQR','LQG')
    end
    
    %Including the actuator dynamics
    
    %lqr
    Klqr = Kc_pitch;
    Klqr.OutputName = 'elev_cmd';
    Klqe_lon.InputName{1}  = 'elev_cmd';
    Gf_lon_lqr_aug = connect(Klqr, Gp_lon_aug,'theta_cmd',{'theta_s','q_s','w_s','u_s'},'elev_cmd');
    Ga_lon_lqr_aug = -getIOTransfer(Gf_lon_lqr_aug,'elev_cmd','elev_cmd','elev_cmd');
    
    %lqg
    Klqr = Kc_pitch;
    Klqr.InputName  = {'u_hat','w_hat','q_hat','theta_hat','theta_cmd'};
    Klqr.OutputName = 'elev_cmd';
    Gf_lon_lqg_aug = connect(Klqr, Klqe_lon, Gp_lon_aug,'theta_cmd',{'theta_s','q_s','w_s','u_s'},'elev_cmd');
    Ga_lon_lqg_aug = -getIOTransfer(Gf_lon_lqg_aug,'elev_cmd','elev_cmd','elev_cmd');
    
    % plots to eval the LQG Design
    if i==1 && flag.lqg_eval == 1
        h = gcf;
        figure()
        bode(Ga_lon_lqr_aug)
        figure()
        step(Gf_lon_lqr_aug)
    end
    
    if flag.lqg_eval == 1
        figure(h.Number +1)
        hold on
        bode(Ga_lon_lqg_aug)
        figure(h.Number +2)
        hold on
        step(Gf_lon_lqg_aug)
    end
    
end

%Insert the legends
if flag.lqg_eval == 1
    figure(h.Number+1)
    legend(aux_str)
    
    figure(h.Number+2)
    legend(aux_str)
end
