% LQR & DLQR PID Optimization for DC Motor
% Porównanie: Model II rzędu (PID) vs Model I rzędu (PI + D=0)
clear; clc; close all;

%% 1. Parametry układu
Ts = 0.020;         % Sampling time [s]
k_fi = 0.004916;    % Motor constant
b = 0.00000869;     % Friction
R = 7.5;            % Resistance
L = 0.0001;         % Inductance
m_car = 0.5;        
m_wheel = 0.05;     
r_wheel = 0.032;    

% Moment bezwładności
J = (0.5 * m_wheel * r_wheel^2) %+ (m_car * r_wheel^2);

% --- MODEL 1: Pełny (II rzędu - uwzględnia L) ---
Am = [-b/J, k_fi/J; -k_fi/L, -R/L];
Bm = [0; 1/L];

% Wagi LQR dla modelu II rzędu
Q = diag([100, 10, 0.1]); % [całka, prędkość, prąd]
R_weight = 1;

%% 2A. Projekt sterownika DLQR -> PID (Model II rzędu)
sys_m = ss(Am, Bm, [1, 0], 0);
sys_md = c2d(sys_m, Ts, 'zoh');
Ad = sys_md.A;
Bd = sys_md.B;

% Rozszerzenie o integrator: [int_e; omega; i]
A_aug = [1, -Ts, 0; 
         0, Ad(1,1), Ad(1,2);
         0, Ad(2,1), Ad(2,2)];
B_aug = [0; Bd];
B_ref = [Ts; 0; 0]; 

Kd_lqr = dlqr(A_aug, B_aug, Q, R_weight);

% Mapowanie na PID
Ki = -Kd_lqr(1);
Kp = Kd_lqr(2) + (Kd_lqr(3)*b/k_fi); 
Kd = Kd_lqr(3)*J/k_fi;

fprintf('========================================\n');
fprintf('   MODEL PEŁNY (II RZĄD)   \n');
fprintf('========================================\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);

%% 2B. Projekt sterownika DLQR -> PID (Model I rzędu)
% Założenie: L = 0. Równanie: T*w' + w = K*u
denom = (b*R + k_fi^2);
K_stat = 13.77 ;    % Wzmocnienie statyczne
T_const = 2.3;  % Stała czasowa

% Model ciągły [omega] (1 stan)
Am_1 = -1/T_const;
Bm_1 = K_stat/T_const;

% Dyskretyzacja
sys_m1 = ss(Am_1, Bm_1, 1, 0);
sys_md1 = c2d(sys_m1, Ts, 'zoh');
Ad1 = sys_md1.A;
Bd1 = sys_md1.B;

% Rozszerzenie o integrator: [int_e; omega]
A_aug1 = [1, -Ts; 
          0, Ad1];
B_aug1 = [0; Bd1];
B_ref1 = [Ts; 0];

% Wagi LQR dla modelu I rzędu (brak prądu w wagach)
Q1 = diag([100, 10]); 
R_weight1 = 1;

Kd_lqr1 = dlqr(A_aug1, B_aug1, Q1, R_weight1);

% Mapowanie na PID (Dla modelu 1 rzędu)
Ki_1 = -Kd_lqr1(1);
Kp_1 = Kd_lqr1(2); 
Kd_1 = 0; % Z definicji modelu I rzędu: brak dynamiki prądu = brak członu D

fprintf('========================================\n');
fprintf('   MODEL UPROSZCZONY (I RZĄD)    \n');
fprintf('========================================\n');
fprintf('Kp = %.4f\n', Kp_1);
fprintf('Ki = %.4f\n', Ki_1);
fprintf('Kd = %.4f\n', Kd_1);

%% 3. Symulacja zamkniętego układu (POPRAWIONE)
setpoint = 10; % [rad/s]
t_final = 5;

% --- Symulacja Modelu II rzędu ---
C_cl = [0, 1, 0; -Kd_lqr]; % Wyjścia: [omega, u]
sys_cl = ss(A_aug - B_aug*Kd_lqr, B_ref, C_cl, [0;0], Ts);

% POPRAWKA: Pobieramy [y, t] zamiast samego y
[y_sim, t_out] = step(setpoint * sys_cl, t_final); 

% --- Symulacja Modelu I rzędu ---
C_cl1 = [0, 1; -Kd_lqr1]; % Wyjścia: [omega, u]
sys_cl1 = ss(A_aug1 - B_aug1*Kd_lqr1, B_ref1, C_cl1, [0;0], Ts);

% POPRAWKA: Pobieramy [y, t] zamiast samego y
[y_sim1, t_out1] = step(setpoint * sys_cl1, t_final);

%% 4. Wykresy (POPRAWIONE)
figure('Color', 'w', 'Name', 'Porównanie PID (2 rząd) vs PID (1 rząd)');

subplot(2,1,1);
% Używamy t_out na osi X
stairs(t_out, y_sim(:,1), 'b', 'LineWidth', 2); hold on;
stairs(t_out1, y_sim1(:,1), 'g--', 'LineWidth', 2);
yline(setpoint, 'r--', 'Setpoint');
grid on;
legend('PID na modelu II rzędu (z Kd)', 'PID na modelu I rzędu', 'Setpoint', 'Location', 'SouthEast');
title(['Odpowiedź skokowa']);
ylabel('Prędkość [rad/s]');

subplot(2,1,2);
% Używamy t_out na osi X
stairs(t_out, y_sim(:,2), 'r', 'LineWidth', 1.5); hold on;
stairs(t_out1, y_sim1(:,2), 'm--', 'LineWidth', 1.5);
yline(12, 'k:', 'Limit +12V');
yline(-12, 'k:', 'Limit -12V');
grid on;
legend('Sterowanie u (II rząd)', 'Sterowanie u (I rząd)');
title('Sygnał sterujący');
ylabel('Napięcie [V]');
xlabel('Czas [s]');