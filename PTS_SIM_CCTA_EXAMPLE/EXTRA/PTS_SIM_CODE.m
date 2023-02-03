%%%%%%%%%%%%%%%%%%%% PTS - CBFs - PD: SINGLE WAYPOINT %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Carmen Jimenenez Cortes
% 05/05/2022

% UPDATED VERSION FROM THE ONE OUTSIDE THIS FOLDER

% Description: System is initialized at a fixed position, velocity and 
% orientation (pointing towards the waypoint and its safe region). It
% tracks a nominal velocity with a PD controller, but the proportional gain
% for the position is 0. 
% Trajectories compared: PD (unsafe) - PTS - CBF
% Modify the definition of function nu, it only actuates when t is very
% close to T_safe, and it is equal to 1 the rest of the time

clear
close all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%% PROBLEM SET UP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GLOBAL VARIABLES
global options_quadprog options_ode45 obj_handle t0 T_safe dist_p

% FUNCTION OPTIONS
options_quadprog = optimoptions('quadprog','Display','None') % Quadprog
options_ode45 = odeset('MaxStep',1e-2) % ODE45

% SIMULATION PARAMETERS
% General parameters
t0 = 0; % Initial time
T = 150; % Time Horizon
step = 0.01; %time step
actuator_noise = 0; % = 1 introduces random noise

% PTS simulation parameters
T_safe = 10; % Seconds since the waypoint is updated
t_wp_PTS = 0; % Time since the waypoint is updated PTS
t_wp_CBF = 0; % Time since the waypoint is updated CBF
t_wp_PD = 0; % Time since the waypoint is updated PD
T_safe_hat = 0.5*0; % For ramp

% SYSTEM STATE
% Initial conditions
dist_p = 0.5;
x1_0 = 20; %35
x2_0 = 0;
s_0 = -1.5;
theta_0 = 0;
omega_0 = 0.0;

y1_0 = x1_0 + dist_p*cos(theta_0);
y2_0 = x2_0 + dist_p*sin(theta_0);
vx1_0 = s_0*cos(theta_0);
vx2_0 = s_0*sin(theta_0);
vy1_0 = vx1_0 - dist_p*omega_0*sin(theta_0);
vy2_0 = vx2_0 + dist_p*omega_0*cos(theta_0);

x_0 = (x1_0^2 + x2_0^2)^(1/2);
y_0 = (y1_0^2 + y2_0^2)^(1/2);

% State structure
state0.x1 = x1_0;
state0.x2 = x2_0;
state0.s = s_0;
state0.theta = theta_0;
state0.omega = omega_0;

state0_disp.y1 = y1_0;
state0_disp.y2 = y2_0;
state0_disp.vy1 = vy1_0;
state0_disp.vy2 = vy2_0;

output0.x = x_0;
output0.y = y_0;

statesdisp_PTS = [state0_disp];
states_PTS = [state0];
inputs_PTS = [];
outputs_PTS = [output0];

% STD CBFS
% State structure
statesdisp_CBF = [state0_disp];
states_CBF = [state0];
inputs_CBF = [];
outputs_CBF = [output0];

% PD - NO CBF
% State structure
statesdisp_PD = [state0_disp];
states_PD = [state0];
inputs_PD = [];
outputs_PD = [output0];

% CBFs CONFIG
% Class K functions are constants * blow up function
c11 = 1*1.4;
c21 = 1*1.4;
% Blow up
mu_max_abs = 50*0.9; %Saturation value on mu
t_nu_act = 1; % Time before T_safe when nu value is used

% WAYPOINTS (UNSAFE REGIONS & THRESHOLDS)
wp_index_PTS = 1; % Index for the list of points PTS
wp_index_CBF = 1; % Index for the list of points CBF
wp_index_PD = 1; % Index for the list of points PD

CBF_flag = 0; % CBFs start deactivated

% Waypoint 1
s1_1 = 0; % y1 coordinate of circular waypoint
s2_1 = 0; % y2 coordinate of circular waypoint
vs1_1 = -1.0; % y1 velocity of circular waypoint
vs2_1 = 0; % y2 velocity of circular waypoint

% Waypoint 2
s1_2 = 43; % y1 coordinate of circular waypoint
s2_2 = 25; % y2 coordinate of circular waypoint
vs1_2 = 0.866; % y1 velocity of circular waypoint
vs2_2 = 0.5; % y2 velocity of circular waypoint

% Waypoint 3
s1_3 = 43; % y1 coordinate of circular waypoint
s2_3 = -5; % y2 coordinate of circular waypoint
vs1_3 = 0; % y1 velocity of circular waypoint
vs2_3 = -1; % y2 velocity of circular waypoint

s1_list = [s1_1, s1_2, s1_3]; % y1 position of waypoints
s2_list = [s2_1, s2_2, s2_3]; % y2 position of waypoints
vs1_list = [vs1_1, vs1_2, vs1_3]; % vy1 velocity of waypoints
vs2_list = [vs2_1, vs2_2, vs2_3]; % vy2 velocity of waypoints

R_safe = 10; % Outer circle (threshold) radius
r_safe = 1; % Inner circle (safe region) radius

% PARAMETRIC EQUATIONS
% Generate all thresholds and safe regions for waypoints

% Waypoint 1
% Threshold
Rsafe_x_1 = s1_list(1) + R_safe*cos(0:0.001:2*pi);
Rsafe_y_1 = s2_list(1) + R_safe*sin(0:0.001:2*pi);
% Safe Region
rsafe_x_1 = s1_list(1) + r_safe*cos(0:0.001:2*pi);
rsafe_y_1 = s2_list(1) + r_safe*sin(0:0.001:2*pi);

% Waypoint 2
% Threshold
Rsafe_x_2 = s1_list(2) + R_safe*cos(0:0.001:2*pi);
Rsafe_y_2 = s2_list(2) + R_safe*sin(0:0.001:2*pi);
% Safe Region
rsafe_x_2 = s1_list(2) + r_safe*cos(0:0.001:2*pi);
rsafe_y_2 = s2_list(2) + r_safe*sin(0:0.001:2*pi);

% Waypoint 3
% ROUNDED SQUARE
% Threshold
theta_sr = 0:0.001:2*pi; % For rounded squares
for i = 1:length(theta_sr)
    imag_check = R_safe*cos(theta_sr(i))^(2/4);
    if imag(imag_check) ~= 0
        Rsafe_x_3(i) = s1_list(3) - R_safe*abs(cos(theta_sr(i))^(2/4));
    else
        Rsafe_x_3(i) = s1_list(3) + R_safe*cos(theta_sr(i))^(2/4);
    end
end

for i = 1:length(theta_sr)
    imag_check = R_safe*sin(theta_sr(i))^(2/4);
    if imag(imag_check) ~= 0
        Rsafe_y_3(i) = s2_list(3) - R_safe*abs(sin(theta_sr(i))^(2/4));
    else
        Rsafe_y_3(i) = s2_list(3) + R_safe*sin(theta_sr(i))^(2/4);
    end
end

% Safe region
for i = 1:length(theta_sr)
    imag_check = r_safe*cos(theta_sr(i))^(2/4);
    if imag(imag_check) ~= 0
        rsafe_x_3(i) = s1_list(3) - r_safe*abs(cos(theta_sr(i))^(2/4));
    else
        rsafe_x_3(i) = s1_list(3) + r_safe*cos(theta_sr(i))^(2/4);
    end
end

for i = 1:length(theta_sr)
    imag_check = r_safe*sin(theta_sr(i))^(2/4);
    if imag(imag_check) ~= 0
        rsafe_y_3(i) = s2_list(3) - r_safe*abs(sin(theta_sr(i))^(2/4));
    else
        rsafe_y_3(i) = s2_list(3) + r_safe*sin(theta_sr(i))^(2/4);
    end
end

% PTS CBFs INIT
% CBFs ON, PTS OFF
h1_act_PTS = 1;
PTS_h1 = 0;
h2_act_PTS = 1;
PTS_h2 = 0;
h3_act_PTS = 1;
PTS_h3 = 0;

% STD CBFs
% CBFs ON
h1_act_CBF = 1;
h2_act_CBF = 1;
h3_act_CBF = 1;

% NOMINAL CONTROLLER -> Programmed inside the loop
% PD to drive the displaced point to the waypoint (s1,s2) with velocity (vs1,vs2)
% u1_nom = -ky1*(y1 - s1) - kv1*(vy1 - vs1)
% u2_nom = -ky2*(y2 - s2) - kv2*(vy2 - vs2)

ky1 = 0; % We only track velocity
ky1d = 10;
ky2 = ky1;
ky2d = ky1d;

u1nom_sat = 40; % Control input saturation
u2nom_sat = 40; % Control input saturation

% END OF MISSION FLAG
end_flag_PTS = 0;
end_flag_CBF = 0;
end_flag_PD = 0;
% MEMORIES FOR DEBUG
% Blow up functions memories
mu_1_sat = [];
mu_1_sat_time = [];
mu_2_sat = [];
mu_2_sat_time = [];
mu_3_sat = [];
mu_3_sat_time = [];
mu_1_mem = [];
mu_2_mem = [];
mu_3_mem = [];


% CBFs MEMORIES
% PTS CBFs
h1_mem_PTS = []; % h1 value in simulation
psi1_mem_PTS = []; % psi1 value in simulation
h1_act_mem_PTS = [];
psi1_act_mem_PTS = [];
PTS_h1_mem_PTS = [];
h2_mem_PTS = []; % h2 value in simulation
psi2_mem_PTS = []; % psi2 value in simulation
h2_act_mem_PTS = [];
psi2_act_mem_PTS = [];
PTS_h2_mem_PTS = [];
h3_mem_PTS = []; % h3 value in simulation
psi3_mem_PTS = []; % psi3 value in simulation
h3_act_mem_PTS = [];
psi3_act_mem_PTS = [];
PTS_h3_mem_PTS = [];

% STD CBFs
h1_mem_CBF = []; % h1 value in simulation
psi1_mem_CBF = []; % psi1 value in simulation
h1dot_mem_PTS = [];
h1_act_mem_CBF = [];
psi1_act_mem_CBF = [];
h2_mem_CBF = []; % h2 value in simulation
psi2_mem_CBF = []; % psi2 value in simulation
h2dot_mem_PTS = [];
h2_act_mem_CBF = [];
psi2_act_mem_CBF = [];
h3_mem_CBF = []; % h3 value in simulation
psi3_mem_CBF = []; % psi3 value in simulation
h3dot_mem_PTS = [];
h3_act_mem_CBF = [];
psi3_act_mem_CBF = [];

% Extra ones
u_nom_mem_PTS = [];
u_nom_mem_CBF = [];
u_nom_mem_PD = [];
g_ramp_mem = [];
t_wp_active_mem_PTS = [];
t_wp_active_mem_CBF = [];
t_wp_active_mem_PD = [];
vs_mem_PTS = [];
theta_mem_PTS = [];
vs_mem_CBF = [];
theta_mem_CBF = [];
vs_mem_PD = [];
theta_mem_PD = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIMULATION
wp_threshold_flag_PTS = 0;
wp_threshold_flag_CBF = 0;
wp_threshold_flag_PD = 0;

for t = step:step:T

    % Save time for updating waypoint
    t_wp_PTS = t_wp_PTS + step;
    t_wp_CBF = t_wp_CBF + step;
    t_wp_PD = t_wp_PD + step;

    % Current waypoint coordinates
    % PTS
    s1_PTS = s1_list(wp_index_PTS);
    s2_PTS = s2_list(wp_index_PTS);

    % CBF
    s1_CBF = s1_list(wp_index_CBF);
    s2_CBF = s2_list(wp_index_CBF);

    % PD
    s1_PD = s1_list(wp_index_PD);
    s2_PD = s2_list(wp_index_PD);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Current State
    currstate_PTS = states_PTS(end);
    currstatedisp_PTS = statesdisp_PTS(end);
    curroutput_PTS = outputs_PTS(end);

    % Debug mem
    h1_act_mem_PTS = [h1_act_mem_PTS; h1_act_PTS];
    PTS_h1_mem_PTS = [PTS_h1_mem_PTS; PTS_h1];

    % Check if PTS has to be activated
    if wp_index_PTS == 1
        if ((currstatedisp_PTS.y1 - s1_1)^2 + (currstatedisp_PTS.y2 - s2_1)^2) <= R_safe^2
            if wp_threshold_flag_PTS == 0
                % Register time system enters threshold
                t_wp_PTS = 0;
                wp_threshold_flag_PTS = 1;
                t_wp_active_mem_PTS = [t_wp_active_mem_PTS; t];
                PTS_h1 = 1;
                PTS_h2 = 0;
                PTS_h3 = 0;
            end
            if (t0 <= t_wp_PTS)&&(t_wp_PTS < (t0+T_safe))
                % Safe time hasn't expired
                h1_act = 1;
                h2_act = 1;
                h3_act = 1;
            elseif t_wp_PTS >= (t0+T_safe)
                % Safe time expired
                h1_act = 0;
                h2_act = 1;
                h3_act = 1;
            end
        end
    elseif wp_index_PTS == 2
        if ((currstatedisp_PTS.y1 - s1_2)^2 + (currstatedisp_PTS.y2 - s2_2)^2) <= R_safe^2
            if wp_threshold_flag_PTS == 0
                % Register time system enters threshold
                t_wp_PTS = 0;
                wp_threshold_flag_PTS = 1;
                t_wp_active_mem_PTS = [t_wp_active_mem_PTS; t];
                PTS_h1 = 0;
                PTS_h2 = 1;
                PTS_h3 = 0;
            end
            if (t0 <= t_wp_PTS)&&(t_wp_PTS < (t0+T_safe))
                % Safe time hasn't expired
                h1_act = 1;
                h2_act = 1;
                h3_act = 1;
            elseif t_wp_PTS >= (t0+T_safe)
                % Safe time expired
                h1_act = 1;
                h2_act = 0;
                h3_act = 1;
            end
        end
    elseif wp_index_PTS == 3
        if ((currstatedisp_PTS.y1 - s1_3)^4 + (currstatedisp_PTS.y2 - s2_3)^4) <= R_safe^4
            if wp_threshold_flag_PTS == 0
                % Register time system enters threshold
                t_wp_PTS = 0;
                wp_threshold_flag_PTS = 1;
                t_wp_active_mem_PTS = [t_wp_active_mem_PTS; t];
                PTS_h1 = 0;
                PTS_h2 = 0;
                PTS_h3 = 1;
            end
            if (t0 <= t_wp_PTS)&&(t_wp_PTS < (t0+T_safe))
                % Safe time hasn't expired
                h1_act = 1;
                h2_act = 1;
                h3_act = 1;
            elseif t_wp_PTS >= (t0+T_safe)
                % Safe time expired
                h1_act = 1;
                h2_act = 1;
                h3_act = 0;
            end
        end
    end

    % Bow Up Function: Use t_wp so diference = 0 at T_safe
    if (PTS_h1 == 1)||(PTS_h2 == 1)||(PTS_h3 == 1)
        t_rem = T_safe - t + t_wp_active_mem_PTS(wp_index_PTS);
        if (t_rem <= t_nu_act)
            nu_main = compute_nu2(t_nu_act,T_safe+t_wp_active_mem_PTS(wp_index_PTS),t);
        else
            nu_main = 1;
        end    
        mu_1 = 1/nu_main;
        if abs(mu_1) >= mu_max_abs
            mu_1 = mu_max_abs*sign(mu_1);
        end
        mu_2 = 1/nu_main^2;
        if mu_2 >= mu_max_abs
            mu_2 = mu_max_abs;
        end
        mu_3 = 1/nu_main^3;
        if abs(mu_3) >= mu_max_abs
            mu_3 = mu_max_abs*sign(mu_3);
        end
    else
        mu_1 = 1;
        mu_2 = 1;
        mu_3 = 1;
    end

    % Save values in memory
    mu_1_mem = [mu_1_mem; mu_1];
    mu_2_mem = [mu_2_mem; mu_2];
    mu_3_mem = [mu_3_mem; mu_3];

    % Check if waypoint has to be updated
    if wp_index_PTS == 1
        if ((currstatedisp_PTS.y1 - s1_1)^2 < 0.01)&&((currstatedisp_PTS.y2 - s2_1)^2 < 0.01)&&(wp_index_PTS<length(s1_list))
            % Update waypoint
            wp_index_PTS = wp_index_PTS + 1;
            % Reset time for safety
            t_wp_PTS = 0;
            wp_threshold_flag_PTS = 0;
            PTS_h1 = 0;
        end
    elseif wp_index_PTS == 2
        if ((currstatedisp_PTS.y1 - s1_2)^2 < 0.01)&&((currstatedisp_PTS.y2 - s2_2)^2 < 0.01)&&(wp_index_PTS<length(s1_list))
            % Update waypoint
            wp_index_PTS = wp_index_PTS + 1;
            % Reset time for safety
            t_wp_PTS = 0;
            wp_threshold_flag_PTS = 0;
            PTS_h2 = 0;
        end
    elseif wp_index_PTS == 3
        if ((currstatedisp_PTS.y1 - s1_3)^4 < 0.01)&&((currstatedisp_PTS.y2 - s2_3)^4 < 0.01)&&(wp_index_PTS<=length(s1_list)&&(end_flag_PTS == 0))
            % Update waypoint
%             wp_index_PTS = wp_index_PTS + 1;
            % Reset time for safety
%             t_wp_PTS = 0;
%             wp_threshold_flag_PTS = 0;
            PTS_h3 = 0;
            end_flag_PTS = 1;
        end
    end

    % Virtual nominal Controller
    % Update velocity - Quadrant
    pos1_PTS = s1_PTS - currstatedisp_PTS.y1;
    pos2_PTS = s2_PTS - currstatedisp_PTS.y2;
    if pos1_PTS >=0
       % First or fourth quadrant
       if pos2_PTS >=0
           %First quadrant
           theta_PTS = atan(abs(pos2_PTS)/abs(pos1_PTS));
       else
           % Fourth quadrant
           theta_PTS = 2*pi - atan(abs(pos2_PTS)/abs(pos1_PTS));
       end
    else
        % Second or third quadrant
        if pos2_PTS >=0
            % Second quadrant
            theta_PTS = pi - atan(abs(pos2_PTS)/abs(pos1_PTS));
        else
            % Third quadrant
            theta_PTS = pi + atan(abs(pos2_PTS)/abs(pos1_PTS));
        end
    end
    theta_mem_PTS = [theta_mem_PTS;theta_PTS];

    % Update target velocity
    vs1_PTS = cos(theta_PTS);
    vs2_PTS = sin(theta_PTS);
    vs_new_PTS = [vs1_PTS,vs2_PTS];
    vs_mem_PTS = [vs_mem_PTS; vs_new_PTS];

    % Nominal Controller
    u1_nom_PTS = -ky1d*(currstatedisp_PTS.vy1 - vs1_PTS);
    u2_nom_PTS = -ky2d*(currstatedisp_PTS.vy2 - vs2_PTS);
    
    if end_flag_PTS == 1
        u1_nom_PTS = -ky1d*(currstatedisp_PTS.vy1 - 0);
        u2_nom_PTS = -ky2d*(currstatedisp_PTS.vy2 - 0);
    end
    
%     if abs(u1_nom_PTS) >= abs(u1nom_sat)
%         u1_nom_PTS = sign(u1_nom_PTS)*u1nom_sat;
%     end
% 
%     if abs(u2_nom_PTS) >= abs(u2nom_sat)
%         u2_nom_PTS = sign(u2_nom_PTS)*u2nom_sat;
%     end 

    u_nom_PTS = [u1_nom_PTS; u2_nom_PTS];
    u_nom_mem_PTS = [u_nom_mem_PTS, u_nom_PTS];

    % Ramp Function
    if (((t0 + T_safe) <= t_wp_PTS)&&(t_wp_PTS <= (T_safe + T_safe_hat)))
        nu_ramp = compute_nu(t_wp_PTS-t0-T_safe, T_safe_hat);
        g_ramp = 1 - nu_ramp^2;
        g_ramp_mem = [g_ramp_mem;g_ramp];
    else
        g_ramp = 1;
        g_ramp_mem = [g_ramp_mem;g_ramp];
    end

    % PTS verification
    if PTS_h1 == 1
        % PTS is executing
        mu_1_1 = mu_1;
        mu_2_1 = mu_2;
    else
        mu_1_1 = 1;
        mu_2_1 = 1;
    end

    h1_PTS = (currstatedisp_PTS.y1 - s1_1)^2 + (currstatedisp_PTS.y2 - s2_1)^2 - r_safe^2;
    h1dot_PTS = 2*((currstatedisp_PTS.y1 - s1_1)*currstatedisp_PTS.vy1 + (currstatedisp_PTS.y2 - s2_1)*currstatedisp_PTS.vy2);
    psi1_PTS = h1dot_PTS + c11*mu_2_1*h1_PTS;
    h1_mem_PTS = [h1_mem_PTS; h1_PTS];
    h1dot_mem_PTS = [h1dot_mem_PTS; h1dot_PTS];
    psi1_mem_PTS = [psi1_mem_PTS; psi1_PTS];

    % Circle quad prog elements
    A_qp_11_PTS = 2*(currstatedisp_PTS.y1 - s1_1);
    A_qp_12_PTS = 2*(currstatedisp_PTS.y2 - s2_1);

    if PTS_h1 == 1
        b_qp_1_PTS = [2*(currstatedisp_PTS.vy1^2 + currstatedisp_PTS.vy2^2)+...
                      c11*(mu_2_2*mu_1_1/T_safe*h1_PTS + mu_2_1*h1dot_PTS) + c21*mu_2_1*psi1_PTS];
    else
        b_qp_1_PTS = [2*(currstatedisp_PTS.vy1^2 + currstatedisp_PTS.vy2^2)+...
                      c11*h1dot_PTS + c21*psi1_PTS];
    end

    % Waypoint 2
    if PTS_h2 == 1
        % PTS is executing
        mu_1_2 = mu_1;
        mu_2_2 = mu_2;
    else
        mu_1_2 = 1;
        mu_2_2 = 1;
    end

    h2_PTS = (currstatedisp_PTS.y1 - s1_2)^2 + (currstatedisp_PTS.y2 - s2_2)^2 - r_safe^2;
    h2dot_PTS = 2*((currstatedisp_PTS.y1 - s1_2)*currstatedisp_PTS.vy1 + (currstatedisp_PTS.y2 - s2_2)*currstatedisp_PTS.vy2);
    psi2_PTS = h2dot_PTS + c11*mu_2_2*h2_PTS;
    h2_mem_PTS = [h2_mem_PTS; h2_PTS];
    h2dot_mem_PTS = [h2dot_mem_PTS; h2dot_PTS];
    psi2_mem_PTS = [psi2_mem_PTS; psi2_PTS];

    % Circle quad prog elements
    A_qp_21_PTS = 2*(currstatedisp_PTS.y1 - s1_2);
    A_qp_22_PTS = 2*(currstatedisp_PTS.y2 - s2_2);

    if PTS_h2 == 1
        b_qp_2_PTS = [2*(currstatedisp_PTS.vy1^2 + currstatedisp_PTS.vy2^2)+...
                      c11*(mu_2_2*mu_1_2/T_safe*h2_PTS + mu_2_2*h2dot_PTS) + c21*mu_2_2*psi2_PTS];
    else
        b_qp_2_PTS = [2*(currstatedisp_PTS.vy1^2 + currstatedisp_PTS.vy2^2)+...
                      c11*h2dot_PTS + c21*psi2_PTS];
    end

    % Waypoint 3
    % PTS verification
    if PTS_h3 == 1
        % PTS is executing
        mu_1_3 = mu_1;
        mu_2_3 = mu_2;
    else
        mu_1_3 = 1;
        mu_2_3 = 1;
    end

    h3 = (currstatedisp_PTS.y1 - s1_3)^4 + (currstatedisp_PTS.y2 - s2_3)^4 - r_safe^4;
    h3dot = 4*((currstatedisp_PTS.y1 - s1_3)^3*currstatedisp_PTS.vy1 + (currstatedisp_PTS.y2 - s2_3)^3*currstatedisp_PTS.vy2);
    psi3 = h3dot + c11*mu_2_3*h3;
    h3_mem_PTS = [h3_mem_PTS; h3];
    psi3_mem_PTS = [psi3_mem_PTS; psi3];

    % Rounded circle quad prog solution
    A_qp_31_PTS = 4*(currstatedisp_PTS.y1 - s1_3)^3;
    A_qp_32_PTS = 4*(currstatedisp_PTS.y2 - s2_3)^3;

    if PTS_h3 == 1
        b_qp_3_PTS = [12*(currstatedisp_PTS.y1 - s1_3)^2*currstatedisp_PTS.vy1^2 + 12*(currstatedisp_PTS.y2 - s2_3)^2*currstatedisp_PTS.vy2^2 + ...
                  c11*(mu_2_3*mu_1_3/T_safe*h3 + mu_2_3*h3dot) + c21*mu_2_3*psi3];
    else
        b_qp_3_PTS = [12*(currstatedisp_PTS.y1 - s1_3)^2*currstatedisp_PTS.vy1^2 + 12*(currstatedisp_PTS.y2 - s2_3)^2*currstatedisp_PTS.vy2^2 + ...
                  c11*h3dot + c21*psi3];
    end

    % Quad prog elements to solve
    A_qp_PTS = -[A_qp_11_PTS,A_qp_12_PTS;A_qp_21_PTS,A_qp_22_PTS;A_qp_31_PTS,A_qp_32_PTS];
    b_qp_PTS = [b_qp_1_PTS; b_qp_2_PTS; b_qp_3_PTS];
    H_qp_PTS = [1 0;0 1];
    f_qp_PTS = -2*[u1_nom_PTS ; u2_nom_PTS];
    [u_applied_PTS, fval_PTS, exit_flag_PTS] = quadprog(H_qp_PTS, double(f_qp_PTS), A_qp_PTS, b_qp_PTS, [],[], [], [], [], options_quadprog);

    % Ramp to ensure continuity at t = t0 + T
    if (t0 <= t_wp_PTS)&&(t_wp_PTS < (t0+T_safe))
        u_applied_PTS = u_applied_PTS;
    else
        u_applied_PTS = u_nom_PTS*g_ramp;
    end

    % Update state PTS
    [nextstate_PTS, nextstatedisp_PTS] = stateupdate_ode(currstate_PTS, currstatedisp_PTS, u_applied_PTS, step, actuator_noise,options_ode45);
%     nextstatedisp
    states_PTS = [states_PTS, nextstate_PTS];
    statesdisp_PTS = [statesdisp_PTS, nextstatedisp_PTS];
    inputs_PTS = [inputs_PTS, u_applied_PTS];
    output_y_PTS = (nextstatedisp_PTS.y1^2 + nextstatedisp_PTS.y2^2)^(1/2);
    output_x_PTS = (nextstate_PTS.x1^2 + nextstate_PTS.x2^2)^(1/2);
    outputs_PTS.y = [outputs_PTS.y; output_y_PTS];
    outputs_PTS.x = [outputs_PTS.x; output_x_PTS];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CBF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Current State
    currstate_CBF = states_CBF(end);
    currstatedisp_CBF = statesdisp_CBF(end);
    curroutput_CBF = outputs_CBF(end);

    % Check if timer has to be activated
    if wp_index_CBF == 1
        if ((currstatedisp_CBF.y1 - s1_1)^2 + (currstatedisp_CBF.y2 - s2_1)^2) <= R_safe^2
            if wp_threshold_flag_CBF == 0
                % Reset time for safety
                t_wp_CBF = 0;
                wp_threshold_flag_CBF = 1;
                t_wp_active_mem_CBF = [t_wp_active_mem_CBF; t];
            end
            if (t0 <= t_wp_CBF)&&(t_wp_CBF < (t0+T_safe))
                h1_act_CBF = 1;
                h2_act_CBF = 1;
            elseif t_wp_CBF >= (t0+T_safe)
                h1_act_CBF = 0;
                h2_act_CBF = 1;
            end
        end
    elseif wp_index_CBF == 2
        if ((currstatedisp_CBF.y1 - s1_2)^2 + (currstatedisp_CBF.y2 - s2_2)^2) <= R_safe^2
            if wp_threshold_flag_CBF == 0
                % Reset time for safety
                t_wp_CBF = 0;
                wp_threshold_flag_CBF = 1;
                t_wp_active_mem_CBF = [t_wp_active_mem_CBF; t];
            end
            if (t0 <= t_wp_CBF)&&(t_wp_CBF < (t0+T_safe))
                h1_act_CBF = 1;
                h2_act_CBF = 1;
            elseif t_wp_CBF >= (t0+T_safe)
                h1_act_CBF = 1;
                h2_act_CBF = 0;
            end
        end
    elseif wp_index_CBF == 3
        if ((currstatedisp_CBF.y1 - s1_3)^4 + (currstatedisp_CBF.y2 - s2_3)^4) <= R_safe^4
            if wp_threshold_flag_CBF == 0
                % Reset time for safety
                t_wp_CBF = 0;
                wp_threshold_flag_CBF = 1;
                t_wp_active_mem_CBF = [t_wp_active_mem_CBF; t];
            end
            if (t0 <= t_wp_CBF)&&(t_wp_CBF < (t0+T_safe))
                h1_act_CBF = 1;
                h2_act_CBF = 1;
                h3_act_CBF = 1;
            elseif t_wp_CBF >= (t0+T_safe)
                h1_act_CBF = 1;
                h2_act_CBF = 1;
                h3_act_CBF = 0;
            end
        end
    end

    % Check if waypoint has to be updated
    if wp_index_CBF == 1
        if ((currstatedisp_CBF.y1 - s1_1)^2 < 0.01)&&((currstatedisp_CBF.y2 - s2_1)^2 < 0.01)&&(wp_index_CBF<length(s1_list))
            % Update waypoint
            wp_index_CBF = wp_index_CBF + 1;
            % Reset time for safety
            t_wp_CBF = 0;
            wp_threshold_flag_CBF = 0;
        end
    elseif wp_index_CBF == 2
        if ((currstatedisp_CBF.y1 - s1_2)^2 < 0.01)&&((currstatedisp_CBF.y2 - s2_2)^2 < 0.01)&&(wp_index_CBF<length(s1_list))
            % Update waypoint
            wp_index_CBF = wp_index_CBF + 1;
            % Reset time for safety
            t_wp_CBF = 0;
            wp_threshold_flag_CBF = 0;
        end
    elseif wp_index_CBF == 3
        if ((currstatedisp_CBF.y1 - s1_3)^4 < 0.01)&&((currstatedisp_CBF.y2 - s2_3)^4 < 0.01)&&(wp_index_CBF<=length(s1_list)&&(end_flag_CBF == 0))
            % Update waypoint
%             wp_index_CBF = wp_index_CBF + 1;
            % Reset time for safety
%             t_wp_CBF = 0;
%             wp_threshold_flag_CBF = 0;
            end_flag_CBF = 1;
        end
    end

    % Virtual nominal controller
    % Update velocity - Quadrant
    pos1_CBF = s1_CBF - currstatedisp_CBF.y1;
    pos2_CBF = s2_CBF - currstatedisp_CBF.y2;
    if pos1_CBF >=0
       % First or fourth quadrant
       if pos2_CBF >=0
           %First quadrant
           theta_CBF = atan(abs(pos2_CBF)/abs(pos1_CBF));
       else
           % Fourth quadrant
           theta_CBF = 2*pi - atan(abs(pos2_CBF)/abs(pos1_CBF));
       end
    else
        % Second or third quadrant
        if pos2_CBF >=0
            % Second quadrant
            theta_CBF = pi - atan(abs(pos2_CBF)/abs(pos1_CBF));
        else
            % Third quadrant
            theta_CBF = pi + atan(abs(pos2_CBF)/abs(pos1_CBF));
        end
    end
    theta_mem_CBF = [theta_mem_CBF;theta_CBF];

    % Update target velocity
    vs1_CBF = cos(theta_CBF);
    vs2_CBF = sin(theta_CBF);
    vs_new_CBF = [vs1_CBF,vs2_CBF];
    vs_mem_CBF = [vs_mem_CBF; vs_new_CBF];

    % Nominal Controller
    u1_nom_CBF = - ky1d*(currstatedisp_CBF.vy1 - vs1_CBF);
    u2_nom_CBF = - ky2d*(currstatedisp_CBF.vy2 - vs2_CBF);
    if end_flag_CBF == 1
        u1_nom_CBF = - ky1d*(currstatedisp_CBF.vy1 - 0);
        u2_nom_CBF = - ky2d*(currstatedisp_CBF.vy2 - 0);
    end

%     if abs(u1_nom_CBF) >= abs(u1nom_sat)
%         u1_nom_CBF = sign(u1_nom_CBF)*u1nom_sat;
%     end
% 
%     if abs(u2_nom_CBF) >= abs(u2nom_sat)
%         u2_nom_CBF = sign(u2_nom_CBF)*u2nom_sat;
%     end 
    
    u_nom_CBF = [u1_nom_CBF; u2_nom_CBF];
    u_nom_mem_CBF = [u_nom_mem_CBF, u_nom_CBF];

    % CBFs
    % Waypoint 1
    h1_CBF = (currstatedisp_CBF.y1 - s1_1)^2 + (currstatedisp_CBF.y2 - s2_1)^2 - r_safe^2;
    h1dot_CBF = 2*((currstatedisp_CBF.y1 - s1_1)*currstatedisp_CBF.vy1 + (currstatedisp_CBF.y2 - s2_1)*currstatedisp_CBF.vy2);
    psi1_CBF = h1dot_CBF + c11*h1_CBF;
    h1_mem_CBF = [h1_mem_CBF; h1_CBF];
    psi1_mem_CBF = [psi1_mem_CBF; psi1_CBF];

    % Circle quad prog elements
    A_qp_11_CBF = 2*(currstatedisp_CBF.y1 - s1_1);
    A_qp_12_CBF = 2*(currstatedisp_CBF.y2 - s2_1);
    b_qp_1_CBF = [2*(currstatedisp_CBF.vy1^2 + currstatedisp_CBF.vy2^2)+...
                  c11*h1dot_CBF + c21*psi1_CBF];

    % Waypoint 2
    h2_CBF = (currstatedisp_CBF.y1 - s1_2)^2 + (currstatedisp_CBF.y2 - s2_2)^2 - r_safe^2;
    h2dot_CBF = 2*((currstatedisp_CBF.y1 - s1_2)*currstatedisp_CBF.vy1 + (currstatedisp_CBF.y2 - s2_2)*currstatedisp_CBF.vy2);
    psi2_CBF = h2dot_CBF + c11*h2_CBF;
    h2_mem_CBF = [h2_mem_CBF; h2_CBF];
    psi2_mem_CBF = [psi2_mem_CBF; psi2_CBF];

    % Circle quad prog elements
    A_qp_21_CBF = 2*(currstatedisp_CBF.y1 - s1_2);
    A_qp_22_CBF = 2*(currstatedisp_CBF.y2 - s2_2);
    b_qp_2_CBF = [2*(currstatedisp_CBF.vy1^2 + currstatedisp_CBF.vy2^2)+...
                  c11*h2dot_CBF + c21*psi2_CBF];

    % Waypoint 3
    h3_CBF = (currstatedisp_CBF.y1 - s1_3)^4 + (currstatedisp_CBF.y2 - s2_3)^4 - r_safe^4;
    h3dot_CBF = 4*((currstatedisp_CBF.y1 - s1_3)^3*currstatedisp_CBF.vy1 + (currstatedisp_CBF.y2 - s2_3)^3*currstatedisp_CBF.vy2);
    psi3_CBF = h3dot_CBF + c11*h3_CBF;
    h3_mem_CBF = [h3_mem_CBF; h3_CBF];
    psi3_mem_CBF = [psi3_mem_CBF; psi3_CBF];

    % Rounded circle quad prog solution
    A_qp_31_CBF = 4*(currstatedisp_CBF.y1 - s1_3)^3;
    A_qp_32_CBF = 4*(currstatedisp_CBF.y2 - s2_3)^3;
    b_qp_3_CBF = [12*(currstatedisp_CBF.y1 - s1_3)^2*currstatedisp_CBF.vy1^2 + 12*(currstatedisp_CBF.y2 - s2_3)^2*currstatedisp_CBF.vy2^2 + ...
                  c11*h3dot_CBF + c21*psi3_CBF];

    A_qp_CBF = -[A_qp_11_CBF,A_qp_12_CBF;A_qp_21_CBF,A_qp_22_CBF;A_qp_31_CBF,A_qp_32_CBF];
    b_qp_CBF = [b_qp_1_CBF; b_qp_2_CBF; b_qp_3_CBF];
    H_qp_CBF = [1 0;0 1];
    f_qp_CBF = -2*[u1_nom_CBF ; u2_nom_CBF];
    [u_applied_CBF, fval_CBF, exit_flag_CBF] = quadprog(H_qp_CBF, double(f_qp_CBF), A_qp_CBF, b_qp_CBF, [],[], [], [], [], options_quadprog);

    % Deactivate barrier at t = t0 + T
    if (t0 <= t_wp_CBF)&&(t_wp_CBF < (t0+T_safe))
        u_applied_CBF = u_applied_CBF;
    else
        u_applied_CBF = u_nom_CBF;
    end

    % Update state CBF
    [nextstate_CBF, nextstatedisp_CBF] = stateupdate_ode(currstate_CBF, currstatedisp_CBF, u_applied_CBF, step, actuator_noise,options_ode45);
%     nextstatedisp_CBF
    states_CBF = [states_CBF, nextstate_CBF];
    statesdisp_CBF = [statesdisp_CBF, nextstatedisp_CBF];
    inputs_CBF = [inputs_CBF, u_applied_CBF];
    output_y_CBF = (nextstatedisp_CBF.y1^2 + nextstatedisp_CBF.y2^2)^(1/2);
    output_x_CBF = (nextstate_CBF.x1^2 + nextstate_CBF.x2^2)^(1/2);
    outputs_CBF.y = [outputs_CBF.y; output_y_CBF];
    outputs_CBF.x = [outputs_CBF.x; output_x_CBF];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PD %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Current State
    currstate_PD = states_PD(end);
    currstatedisp_PD = statesdisp_PD(end);
    curroutput_PD = outputs_PD(end);

%     Check if timer has to be activated
    if wp_index_PD == 1
        if ((currstatedisp_PD.y1 - s1_1)^2 + (currstatedisp_PD.y2 - s2_1)^2) <= R_safe^2
            if wp_threshold_flag_PD == 0
                % Reset time for safety
                t_wp_PD = 0;
                wp_threshold_flag_PD = 1;
                t_wp_active_mem_PD = [t_wp_active_mem_PD; t];
            end
        end
    elseif wp_index_PD == 2
        if ((currstatedisp_PD.y1 - s1_2)^2 + (currstatedisp_PD.y2 - s2_2)^2) <= R_safe^2
            if wp_threshold_flag_PD == 0
                % Reset time for safety
                t_wp_PD = 0;
                wp_threshold_flag_PD = 1;
                t_wp_active_mem_PD = [t_wp_active_mem_PD; t];
            end
        end
    elseif wp_index_PD == 3
        if ((currstatedisp_PD.y1 - s1_3)^4 + (currstatedisp_PD.y2 - s2_3)^4) <= R_safe^4
            if wp_threshold_flag_PD == 0
                % Reset time for safety
                t_wp_PD = 0;
                wp_threshold_flag_PD = 1;
                t_wp_active_mem_PD = [t_wp_active_mem_PD; t];
            end
        end
    end

    % Check if waypoint has to be updated
    if wp_index_PD == 1
        if ((currstatedisp_PD.y1 - s1_1)^2 < 0.01)&&((currstatedisp_PD.y2 - s2_1)^2 < 0.01)&&(wp_index_PD<length(s1_list))
            % Update waypoint
            wp_index_PD = wp_index_PD + 1;
            % Reset time for safety
            t_wp_PD = 0;
            wp_threshold_flag_PD = 0;
        end
    elseif wp_index_PD == 2
        if ((currstatedisp_PD.y1 - s1_2)^2 < 0.01)&&((currstatedisp_PD.y2 - s2_2)^2 < 0.01)&&(wp_index_PD<length(s1_list))
            % Update waypoint
            wp_index_PD = wp_index_PD + 1;
            % Reset time for safety
            t_wp_PD = 0;
            wp_threshold_flag_PD = 0;
        end
    elseif wp_index_PD == 3
        if ((currstatedisp_PD.y1 - s1_3)^4 < 0.01)&&((currstatedisp_PD.y2 - s2_3)^4 < 0.01)&&(wp_index_PD<=length(s1_list)&&(end_flag_PD == 0))
            % Update waypoint
%             wp_index_PD = wp_index_PD + 1;
            % Reset time for safety
%             t_wp_PD = 0;
%             wp_threshold_flag_PD = 0;
            end_flag_PD = 1;
        end        
    end

    % Virtual nominal controller
    % Update velocity - Quadrant
    pos1_PD = s1_PD - currstatedisp_PD.y1;
    pos2_PD = s2_PD - currstatedisp_PD.y2;
    if pos1_PD >=0
       % First or fourth quadrant
       if pos2_PD >=0
           %First quadrant
           theta_PD = atan(abs(pos2_PD)/abs(pos1_PD));
       else
           % Fourth quadrant
           theta_PD = 2*pi - atan(abs(pos2_PD)/abs(pos1_PD));
       end
    else
        % Second or third quadrant
        if pos2_PD >=0
            % Second quadrant
            theta_PD = pi - atan(abs(pos2_PD)/abs(pos1_PD));
        else
            % Third quadrant
            theta_PD = pi + atan(abs(pos2_PD)/abs(pos1_PD));
        end
    end
    theta_mem_PD = [theta_mem_PD;theta_PD];

    % Update target velocity
    vs1_PD = cos(theta_PD);
    vs2_PD = sin(theta_PD);
    vs_new_PD = [vs1_PD,vs2_PD];
    vs_mem_PD = [vs_mem_PD; vs_new_PD];
    u1_nom_PD = - ky1d*(currstatedisp_PD.vy1 - vs1_PD);
    u2_nom_PD = - ky2d*(currstatedisp_PD.vy2 - vs2_PD);
    if end_flag_PD == 1
        u1_nom_PD = - ky1d*(currstatedisp_PD.vy1 - 0);
        u2_nom_PD = - ky2d*(currstatedisp_PD.vy2 - 0);
    end
    
    if abs(u1_nom_PD) >= abs(u1nom_sat)
        u1_nom_PD = sign(u1_nom_PD)*u1nom_sat;
    end

    if abs(u2_nom_PD) >= abs(u2nom_sat)
        u2_nom_PD = sign(u2_nom_PD)*u2nom_sat;
    end

    u_nom_PD = [u1_nom_PD; u2_nom_PD];
    u_applied_PD = u_nom_PD;
    
    u_nom_mem_PD = [u_nom_mem_PD, u_nom_PD];

    % Update state PD
    [nextstate_PD, nextstatedisp_PD] = stateupdate_ode(currstate_PD, currstatedisp_PD, u_applied_PD, step, actuator_noise,options_ode45);
    nextstatedisp_PD
    states_PD = [states_PD, nextstate_PD];
    statesdisp_PD = [statesdisp_PD, nextstatedisp_PD];
    inputs_PD = [inputs_PD, u_applied_PD];
    output_y_PD = (nextstatedisp_PD.y1^2 + nextstatedisp_PD.y2^2)^(1/2);
    output_x_PD = (nextstate_PD.x1^2 + nextstate_PD.x2^2)^(1/2);
    outputs_PD.y = [outputs_PD.y; output_y_PD];
    outputs_PD.x = [outputs_PD.x; output_x_PD];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%% RELEVANT PLOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AUX VARIABLES FOR PLOTS
ts = 0:step:T;

% PTS
y1s_PTS = [statesdisp_PTS.y1];
y2s_PTS = [statesdisp_PTS.y2];
vy1s_PTS = [statesdisp_PTS.vy1];
vy2s_PTS = [statesdisp_PTS.vy2];
thetas_PTS = [states_PTS.theta];
x1s_PTS = [states_PTS.x1];
x2s_PTS = [states_PTS.x2];
outys_PTS = [outputs_PTS.y];
outxs_PTS = [outputs_PTS.x];
input1s_PTS = [0,inputs_PTS(1,:)];
input2s_PTS = [0,inputs_PTS(2,:)];

% CBF
y1s_CBF = [statesdisp_CBF.y1];
y2s_CBF = [statesdisp_CBF.y2];
vy1s_CBF = [statesdisp_CBF.vy1];
vy2s_CBF = [statesdisp_CBF.vy2];
thetas_CBF = [states_CBF.theta];
x1s_CBF = [states_CBF.x1];
x2s_CBF = [states_CBF.x2];
outys_CBF = [outputs_CBF.y];
outxs_CBF = [outputs_CBF.x];
input1s_CBF = [0,inputs_CBF(1,:)];
input2s_CBF = [0,inputs_CBF(2,:)];

% PD
y1s_PD = [statesdisp_PD.y1];
y2s_PD = [statesdisp_PD.y2];
vy1s_PD = [statesdisp_PD.vy1];
vy2s_PD = [statesdisp_PD.vy2];
thetas_PD = [states_PD.theta];
x1s_PD = [states_PD.x1];
x2s_PD = [states_PD.x2];
outys_PD = [outputs_PD.y];
outxs_PD = [outputs_PD.x];
input1s_PD = [0,inputs_PD(1,:)];
input2s_PD = [0,inputs_PD(2,:)];

% OUTPUT
wp1 = sqrt(s1_1^2 + s2_1^2);
wp2 = sqrt(s1_2^2 + s2_2^2);
wp3 = sqrt(s1_3^2 + s2_3^2);
figure()
plot(ts,outys_PTS,'LineWidth',1.5); hold on
plot(ts,outys_CBF,'LineWidth',1.5); hold on
plot(ts,outys_PD,'LineWidth',1.5); hold on
yline(wp1,'--','LineWidth',1.5); hold on
yline(wp2,'--','LineWidth',1.5); hold on
yline(wp3,'--','LineWidth',1.5); hold on
% yline(r_safe,'--','LineWidth',1.5); hold on
% yline(-r_safe,'--','LineWidth',1.5); hold on
% yline(R_safe,'-.','LineWidth',1.5); hold on
% yline(-R_safe,'-.','LineWidth',1.5); hold on
% r_safe2p = r_safe + sqrt(s1_2^2 + s2_2^2);
% r_safe2n = -r_safe + sqrt(s1_2^2 + s2_2^2);
% R_safe2p = R_safe + sqrt(s1_2^2 + s2_2^2);
% R_safe2n = -R_safe + sqrt(s1_2^2 + s2_2^2);
% yline(r_safe2p,'--','LineWidth',1.5); hold on
% yline(r_safe2n,'--','LineWidth',1.5); hold on
% yline(R_safe2p,'--','LineWidth',1.5); hold on
% yline(R_safe2n,'--','LineWidth',1.5); hold on
yline(0,'--','LineWidth',1.5); hold on
xline(t_wp_active_mem_PTS,'r','LineWidth',1.5); hold on
end_of_safe = t_wp_active_mem_PTS + T_safe;
xline(end_of_safe,'b','LineWidth',1.5); hold on
xlabel('$t$','Interpreter','latex');
ylabel('$y$','Interpreter','latex');
legend('PTS','CBF','PD','Interpreter','latex');
ylim([-40 60])

% OUTPUT
figure()
plot(ts,outys_PTS,'LineWidth',1.5); hold on
plot(ts,outys_CBF,'LineWidth',1.5); hold on
plot(ts,outys_PD,'LineWidth',1.5); hold on
yline(r_safe,'--','LineWidth',1.5); hold on
yline(-r_safe,'--','LineWidth',1.5); hold on
yline(R_safe,'-.','LineWidth',1.5); hold on
yline(-R_safe,'-.','LineWidth',1.5); hold on
xline(t_wp_active_mem_PTS,'r','LineWidth',1.5); hold on
end_of_safe = t_wp_active_mem_PTS + T_safe;
xline(end_of_safe,'b','LineWidth',1.5); hold on
xlabel('$t$','Interpreter','latex');
ylabel('$y$','Interpreter','latex');
legend('PTS','CBF','PD','Interpreter','latex');
ylim([-0.5 2.5])
xlim([30 40])

% PTS v CBF v PD
n = 2;
m = 1;

figure()
subplot(n,m,1);
plot(ts,y1s_PTS(1,:),'LineWidth',1.5); hold on
plot(ts,y1s_CBF(1,:),'LineWidth',1.5); hold on
plot(ts,y1s_PD(1,:),'LineWidth',1.5); hold on
grid
xlabel('$t$','Interpreter','latex');
ylabel('$y_1$','Interpreter','latex');
legend('PTS','CBF','PD','Interpreter','latex');
ylim([-40 40])

subplot(n,m,2);
plot(ts,y2s_PTS(1,:),'LineWidth',1.5); hold on
plot(ts,y2s_CBF(1,:),'LineWidth',1.5); hold on
plot(ts,y2s_PD(1,:),'LineWidth',1.5); hold on
grid
xlabel('$t$','Interpreter','latex');
ylabel('$y_2$','Interpreter','latex');
legend('PTS','CBF','PD','Interpreter','latex');
ylim([-40 40])

n = 2;
m = 1;

figure();
subplot(n,m,1);
plot(ts,vy1s_PTS(1,:),'LineWidth',1.5); hold on
plot(ts,vy1s_CBF(1,:),'LineWidth',1.5); hold on
plot(ts,vy1s_PD(1,:),'LineWidth',1.5); hold on
grid
xlabel('$t$','Interpreter','latex');
ylabel('$vy_1$','Interpreter','latex');
legend('PTS','CBF','PD','Interpreter','latex');
ylim([-20 20])

subplot(n,m,2);
plot(ts,vy2s_PTS(1,:),'LineWidth',1.5); hold on
plot(ts,vy2s_CBF(1,:),'LineWidth',1.5); hold on
plot(ts,vy2s_PD(1,:),'LineWidth',1.5); hold on
grid
xlabel('$t$','Interpreter','latex');
ylabel('$vy_2$','Interpreter','latex');
legend('PTS','CBF','PD','Interpreter','latex');
ylim([-20 20])

n = 2;
m = 1;

figure();
subplot(n,m,1);
plot(ts,input1s_PTS,'LineWidth',1.5); hold on
plot(ts,input1s_CBF,'LineWidth',1.5); hold on
plot(ts,input1s_PD,'LineWidth',1.5); hold on
grid
xlabel('$t$','Interpreter','latex');
ylabel('$u_1$','Interpreter','latex');
legend('PTS','CBF','PD','Interpreter','latex');
ylim([-10 10])

subplot(n,m,2);
plot(ts,input2s_PTS,'LineWidth',1.5); hold on
plot(ts,input2s_CBF,'LineWidth',1.5); hold on
plot(ts,input2s_PD,'LineWidth',1.5); hold on
grid
xlabel('$t$','Interpreter','latex');
ylabel('$u_2$','Interpreter','latex');
legend('PTS','CBF','PD','Interpreter','latex');
ylim([-2 2])

% Safe region plot
figure()
plot(y1s_PTS(1,:),y2s_PTS(1,:),'LineWidth',1.5); hold on
plot(y1s_CBF(1,:),y2s_CBF(1,:),'LineWidth',1.5); hold on
plot(y1s_PD(1,:),y2s_PD(1,:),'LineWidth',1.5); hold on
plot(Rsafe_x_1,Rsafe_y_1,'--','LineWidth',1.5,'color','#7E2F8E'); hold on
plot(rsafe_x_1,rsafe_y_1,'-','LineWidth',1.5,'color','#7E2F8E'); hold on
plot(Rsafe_x_2,Rsafe_y_2,'--','LineWidth',1.5,'color','#77AC30'); hold on
plot(rsafe_x_2,rsafe_y_2,'-','LineWidth',1.5,'color','#77AC30'); hold on
plot(Rsafe_x_3,Rsafe_y_3,'--','LineWidth',1.5,'color','#4DBEEE'); hold on
plot(rsafe_x_3,rsafe_y_3,'-','LineWidth',1.5,'color','#4DBEEE'); hold on

xlim([-20 60])
ylim([-20 50])
grid
xlabel('$y_1$','Interpreter','latex');
ylabel('$y_2$','Interpreter','latex');
legend('PTS','CBF','PD','Interpreter','latex');

% CBFs plot
n = 2;
m = 1;

figure()
subplot(n,m,1);
plot(h1_mem_PTS,'LineWidth',1.5); hold on
xlabel('$t$','Interpreter','latex');
ylabel('$h_1$','Interpreter','latex');
yline(0, '--','LineWidth',1); hold on
xline(400, '--','LineWidth',1); hold on

subplot(n,m,2);
plot(psi1_mem_PTS,'LineWidth',1.5); hold on
xlabel('$t$','Interpreter','latex');
ylabel('$psi_1$','Interpreter','latex');
yline(0, '--','LineWidth',1); hold on
xline(400, '--','LineWidth',1); hold on

% Blow up functions plots
n = 1;
m = 3;
figure()
subplot(n,m,1);
plot(mu_1_mem, 'LineWidth',1.5)
xlabel('$t$','Interpreter','latex')
legend('$\mu_1$','Interpreter','latex')

subplot(n,m,2);
plot(mu_2_mem, 'LineWidth',1.5)
xlabel('$t$','Interpreter','latex')
legend('$\mu_2$','Interpreter','latex')

subplot(n,m,3);
plot(mu_3_mem, 'LineWidth',1.5)
xlabel('$t$','Interpreter','latex')
legend('$\mu_3$','Interpreter','latex')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%% SIMULATION FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function nu = compute_nu(t_diff,T_cap)
    nu = (T_cap - t_diff)/T_cap;
end

function nu = compute_nu2(t_margin_nu,T_safe_nu,t)
    nu = T_safe_nu/t_margin_nu - 1/t_margin_nu*t;
end

function dydt = state_derivative(t,y,tau1,tau2)
    global dist_p
    y1dot = y(3);
    y2dot = y(4);
    y1ddot = tau1;
    y2ddot = tau2;
    sdot = -y(7)^2*dist_p + tau1*cos(y(6)) + tau2*sin(y(6));
    thetadot = y(7);
    omegadot = 1/dist_p*(y(7)*y(5) - tau1*sin(y(6)) + tau2*cos(y(6)));
    dydt = [y1dot; y2dot; y1ddot; y2ddot; sdot; thetadot; omegadot];
end

function [statenew, statenewdisp] = stateupdate_ode(state,statedisp,u,step,noise,options_ode45)
    global dist_p
    % State
    y1 = statedisp.y1;
    y2 = statedisp.y2;
    vy1 = statedisp.vy1;
    vy2 = statedisp.vy2;
    s = state.s;
    theta = state.theta;
    omega = state.omega;

    % Inputs
    tau1 = u(1) + noise*randn;
    tau2 = u(2) + noise*randn;
    
    % Build y variable for ode function
    y0 = [y1; y2; vy1; vy2; s; theta; omega];

    % Call ODE45
    [~,y] = ode45(@(t,y) state_derivative(t,y,tau1,tau2),[0,step],y0,options_ode45);
    
    % Build new state from y variable
    statenewdisp.y1 = y(end,1);
    statenewdisp.y2 = y(end,2);
    statenewdisp.vy1 = y(end,3);
    statenewdisp.vy2 = y(end,4);
    statenew.s = y(end,5);
    statenew.theta = y(end,6);
    statenew.omega = y(end,7);

    % Displaced point state
    statenew.x1 = y(end,1) - dist_p*cos(y(end,6));
    statenew.x2 = y(end,2) - dist_p*sin(y(end,6));
end 