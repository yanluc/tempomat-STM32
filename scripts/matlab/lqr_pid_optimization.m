clear; clc; close all;

Ts = 0.020;
k_fi = 0.004916;    
b = 0.00000869;     
R = 7.5;            
L = 0.0001;         
m_car = 0.5;        
m_wheel = 0.05;     
r_wheel = 0.032;
J = (0.5 * m_wheel * r_wheel^2) + (m_car * r_wheel^2);

% --- MODEL 1: Pełny (II rzędu) ---
Am = [-b/J, k_fi/J; -k_fi/L, -R/L];
Bm = [0; 1/L];
Q = diag([100, 10, 0.1]); 
R_weight = 1;

sys_m = ss(Am, Bm, [1, 0], 0);
sys_md = c2d(sys_m, Ts, 'zoh');
Ad = sys_md.A;
Bd = sys_md.B;

A_aug = [1, -Ts, 0; 
         0, Ad(1,1), Ad(1,2);
         0, Ad(2,1), Ad(2,2)];
B_aug = [0; Bd];
B_ref = [Ts; 0; 0]; 
Kd_lqr = dlqr(A_aug, B_aug, Q, R_weight);

Ki = -Kd_lqr(1);
Kp = Kd_lqr(2) + (Kd_lqr(3)*b/k_fi); 
Kd = Kd_lqr(3)*J/k_fi;

fprintf('========================================\n');
fprintf('   MODEL PEŁNY (II RZĄD\n');
fprintf('Kp = %.4f, Ki = %.4f, Kd = %.4f\n', Kp, Ki, Kd);

K_stat = 13.77;    
tau = 2.3;  

Am_1 = -1/tau;
Bm_1 = K_stat/tau;


sys_m1 = ss(Am_1, Bm_1, 1, 0);
sys_md1 = c2d(sys_m1, Ts, 'zoh');
Ad1 = sys_md1.A;
Bd1 = sys_md1.B;

A_aug1 = [1,   -Ts,    0;
          0,   Ad1,    0;
          0,    1,     0];

B_aug1 = [0; Bd1; 0];
B_ref1 = [Ts; 0; 0];

% Wagi LQR
Q1 = diag([100, 10, 1]); % [Całka, Prędkość, Poprzednia Prędkość]
R_weight1 = 1;

Kd_lqr1 = dlqr(A_aug1, B_aug1, Q1, R_weight1);

Ki_1 = -Kd_lqr1(1);
Kd_1 = -Kd_lqr1(3) * Ts;
Kp_1 = Kd_lqr1(2) + Kd_lqr1(3); 

fprintf('========================================\n');
fprintf('   MODEL UPROSZCZONY (I RZĄD)   \n');
fprintf('Kp = %.4f, Ki = %.4f, Kd = %.4f\n', Kp_1, Ki_1, Kd_1);


setpoint = 10; 
t_final = 5;

C_cl = [0, 1, 0; -Kd_lqr]; 
sys_cl = ss(A_aug - B_aug*Kd_lqr, B_ref, C_cl, [0;0], Ts);
[y_sim, t_out] = step(setpoint * sys_cl, t_final); 


C_cl1 = [0, 1, 0; -Kd_lqr1];
sys_cl1 = ss(A_aug1 - B_aug1*Kd_lqr1, B_ref1, C_cl1, [0;0], Ts);
[y_sim1, t_out1] = step(setpoint * sys_cl1, t_final);

figure('Color', 'w', 'Name', 'Porównanie PID');

subplot(2,1,1);
stairs(t_out, y_sim(:,1), 'b', 'LineWidth', 2); hold on;
stairs(t_out1, y_sim1(:,1), 'g--', 'LineWidth', 2);
yline(setpoint, 'r--', 'Setpoint');
grid on;
legend('PID (Model II rzędu)', 'PID (Model I rzędu)', 'Location', 'SouthEast');
title(['Odpowiedź skokowa']);
ylabel('Prędkość [rad/s]');

subplot(2,1,2);
stairs(t_out, y_sim(:,2), 'r', 'LineWidth', 1.5); hold on;
stairs(t_out1, y_sim1(:,2), 'm--', 'LineWidth', 1.5);
yline(12, 'k:', 'Limit +12V');
yline(-12, 'k:', 'Limit -12V');
grid on;
legend('Sterowanie (II rząd)', 'Sterowanie (I rząd)');
title('Sygnał sterujący');
ylabel('Napięcie [V]');
xlabel('Czas [s]');