% LQR & DLQR PID Optimization for DC Motor
clear; clc; close all;

%% 1. Parametry (Dostosowane do Twojego silnika: 10V -> ~227 rad/s)
Ts = 0.020;         % Sampling time [s]
k_fi = 0.004916;    % Motor constant
b = 0.00000869;     % Friction
R = 7.5;            % Resistance
tau = 7.2925;       % Time constant
L = 0.0001;
m_car = 0.5;        
m_wheel = 0.05;     
r_wheel = 0.032;    

% Moment bezwładności
J = (0.5 * m_wheel * r_wheel^2) + (m_car * r_wheel^2);

% Model ciągły silnika [omega; i]
Am = [-b/J, k_fi/J; -k_fi/L, -R/L];
Bm = [0; 1/L];

% Wagi LQR
Q = diag([100, 10, 0.1]); % [całka błędu, prędkość, prąd]
R_weight = 1;

%% 2. Projekt sterownika dyskretnego (DLQR)
sys_m = ss(Am, Bm, [1, 0], 0);
sys_md = c2d(sys_m, Ts, 'zoh');
Ad = sys_md.A;
Bd = sys_md.B;

% Rozszerzenie o integrator: [int_e; omega; i]
% int_e(k) = int_e(k-1) + Ts*(w_set - omega(k))
A_aug = [1, -Ts, 0; 
         0, Ad(1,1), Ad(1,2);
         0, Ad(2,1), Ad(2,2)];
B_aug = [0; Bd];
B_ref = [Ts; 0; 0]; % Wejście wartości zadanej (Setpoint)

Kd_lqr = dlqr(A_aug, B_aug, Q, R_weight);

% Mapowanie na PID (dla sprawozdania)
Ki = -Kd_lqr(1);
Kp = Kd_lqr(2) + (Kd_lqr(3)*b/k_fi); 
Kd = Kd_lqr(3)*J/k_fi;

fprintf('--- Nastawy PID z DLQR (Ts = %d ms) ---\n', Ts*1000);
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n\n', Kd);

%% 3. Symulacja zamkniętego układu
setpoint = 100; % Wartość zadana [rad/s]
t_final = 5;
t = 0:Ts:t_final;

% Budujemy system, który ma dwa wyjścia: [prędkość; napięcie]
% y1 = omega = [0 1 0] * x
% y2 = u = -K * x
C_cl = [0, 1, 0; -Kd_lqr];
D_cl = [0; 0];
sys_cl = ss(A_aug - B_aug*Kd_lqr, B_ref, C_cl, D_cl, Ts);

[y_sim, t_out] = step(setpoint * sys_cl, t_final);

% Rozdzielenie wyników
v_actual = y_sim(:,1);
u_control = y_sim(:,2);

%% 4. Wykresy
figure('Color', 'w', 'Name', 'Symulacja DLQR');

subplot(2,1,1);
stairs(t_out, v_actual, 'b', 'LineWidth', 2); hold on;
yline(setpoint, 'r--', 'Setpoint');
grid on;
title(['Odpowiedź skokowa: ', num2str(setpoint), ' rad/s']);
ylabel('Prędkość [rad/s]');

subplot(2,1,2);
stairs(t_out, u_control, 'r', 'LineWidth', 1.5); hold on;
yline(12, 'k:', 'Limit +12V');
yline(-12, 'k:', 'Limit -12V');
grid on;
title('Sygnał sterujący (Napięcie)');
ylabel('Napięcie [V]');
xlabel('Czas [s]');