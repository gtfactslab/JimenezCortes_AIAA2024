%%%%%%%%%%%%%%%%%% PD: WAYPOINTS TRACKING FLIGHT GEAR %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Carmen Jimenenez Cortes
% 06/21/2022

% Description: System is initialized at a fixed position, velocity and 
% orientation (pointing towards the waypoint and its safe region). It
% tracks a nominal velocity with a PD controller, but the proportional gain
% for the position is 0. 

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
T = 100; % Time Horizon
step = 0.01; %time step
actuator_noise = 0; % = 1 introduces random noise

% PTS simulation parameters
T_safe = 10; % Seconds since the waypoint is updated
t_wp_PD = 0; % Time since the waypoint is updated P

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

% PD - NO CBF
% State structure
statesdisp_PD = [state0_disp];
states_PD = [state0];
inputs_PD = [];
outputs_PD = [output0];

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
end_flag_PD = 0;

% MEMORIES FOR DEBUG
u_nom_mem_PD = [];
t_wp_active_mem_PD = [];
vs_mem_PD = [];
theta_mem_PD = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIMULATION
wp_threshold_flag_PD = 0;

for t = step:step:T

    % Save time for updating waypoint
    t_wp_PD = t_wp_PD + step;

    % Current waypoint coordinates
    % PD
    s1_PD = s1_list(wp_index_PD);
    s2_PD = s2_list(wp_index_PD);
    
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
plot(ts,outys_PD,'LineWidth',1.5); hold on
yline(wp1,'--','LineWidth',1.5); hold on
yline(wp2,'--','LineWidth',1.5); hold on
yline(wp3,'--','LineWidth',1.5); hold on
yline(0,'--','LineWidth',1.5); hold on
xlabel('$t$','Interpreter','latex');
ylabel('$y$','Interpreter','latex');
ylim([-40 60])

% PTS v CBF v PD
n = 2;
m = 1;

figure()
subplot(n,m,1);
plot(ts,y1s_PD(1,:),'LineWidth',1.5); hold on
grid
xlabel('$t$','Interpreter','latex');
ylabel('$y_1$','Interpreter','latex');
ylim([-40 40])

subplot(n,m,2);
plot(ts,y2s_PD(1,:),'LineWidth',1.5); hold on
grid
xlabel('$t$','Interpreter','latex');
ylabel('$y_2$','Interpreter','latex');
ylim([-40 40])

n = 2;
m = 1;

figure();
subplot(n,m,1);
plot(ts,vy1s_PD(1,:),'LineWidth',1.5); hold on
grid
xlabel('$t$','Interpreter','latex');
ylabel('$vy_1$','Interpreter','latex');
ylim([-20 20])

subplot(n,m,2);
plot(ts,vy2s_PD(1,:),'LineWidth',1.5); hold on
grid
xlabel('$t$','Interpreter','latex');
ylabel('$vy_2$','Interpreter','latex');
ylim([-20 20])

n = 2;
m = 1;

figure();
subplot(n,m,1);
plot(ts,input1s_PD,'LineWidth',1.5); hold on
grid
xlabel('$t$','Interpreter','latex');
ylabel('$u_1$','Interpreter','latex');
ylim([-10 10])

subplot(n,m,2);
plot(ts,input2s_PD,'LineWidth',1.5); hold on
grid
xlabel('$t$','Interpreter','latex');
ylabel('$u_2$','Interpreter','latex');
ylim([-2 2])

% Safe region plot
figure()
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