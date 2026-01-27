% LQR vs DLQR Optimization for DC Motor Velocity Control
% Compares continuous vs discrete (20ms) PID parameters
clear; clc; close all;

%% 1. Parameters (SI Units)
Ts = 0.020;         % Sampling time [s]
k_fi = 0.004916;    % Motor constant
b = 0.00000869;     % Friction
R = 7.5;            % Resistance
tau = 7.2925;       % Time constant
m_car = 0.1;        % Car mass
r_wheel = 0.03;     % Wheel radius
L = 0.1;            % Assumed inductance

% Total Inertia J
J = (tau * (k_fi^2 + b*R) / R) + (m_car * r_wheel^2);

%% 2. Continuous LQR
% Continuous state: [int_e; omega; i]
Ac = [0, -1, 0; 
      0, -b/J, k_fi/J;
      0, -k_fi/L, -R/L];
Bc = [0; 0; 1/L];

Q = diag([1000, 10, 1]);
R_weight = 1;

Kc = lqr(Ac, Bc, Q, R_weight);

% Continuous PID extraction
Ki_c = Kc(1);
Kp_c = Kc(2) + (Kc(3) * b / k_fi);
Kd_c = Kc(3) * J / k_fi;

%% 3. Discrete LQR (DLQR)
% Discretize plant only
sys_c = ss(Ac(2:3, 2:3), Bc(2:3), [1, 0], 0);
sys_d = c2d(sys_c, Ts, 'zoh');
Ad = sys_d.A;
Bd = sys_d.B;

% Augmented discrete: [sum_e; omega; i]
A_aug_d = [1, -Ts, 0; 
           0, Ad(1,1), Ad(1,2);
           0, Ad(2,1), Ad(2,2)];
B_aug_d = [0; Bd];

Kd_gain = dlqr(A_aug_d, B_aug_d, Q, R_weight);

% Discrete PID extraction
Ki_d = Kd_gain(1);
Kp_d = Kd_gain(2) + (Kd_gain(3) * b / k_fi);
Kd_d = Kd_gain(3) * J / (k_fi * Ts);

%% 4. Results Comparison
fprintf('--- Continuous LQR PID ---\n');
fprintf('Kp = %.6f, Ki = %.6f, Kd = %.6f\n\n', Kp_c, Ki_c, Kd_c);

fprintf('--- Discrete DLQR PID (Ts = %d ms) ---\n', Ts*1000);
fprintf('Kp = %.6f, Ki = %.6f, Kd = %.6f\n\n', Kp_d, Ki_d, Kd_d);

%% 5. Simulation
t_sim = 0:0.001:3;
sys_cl_c = ss(Ac - Bc*Kc, [1;0;0], [0,1,0], 0);
sys_cl_d = ss(A_aug_d - B_aug_d*Kd_gain, [Ts;0;0], [0,1,0], 0, Ts);

[yc, tc] = step(sys_cl_c, t_sim);

% Discrete simulation
t_discrete = 0:Ts:3;
[yd, td] = step(sys_cl_d, t_discrete);

figure('Name', 'LQR vs DLQR Comparison', 'Color', 'w');
plot(tc, yc, 'r-', 'LineWidth', 1.5); hold on;
stairs(td, yd, 'b', 'LineWidth', 1.5);
yline(1, 'k--');
grid on;
legend('Continuous LQR', 'Discrete DLQR (20ms)');
title('Velocity Control: Continuous vs Discrete LQR');
ylabel('Angular Velocity [rad/s]');
xlabel('Time [s]');
