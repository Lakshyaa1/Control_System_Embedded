
clc;
clear;
close all;

% Plant parameters
fn = 1.5e3;
wn = 2*pi*fn;
zeta = 0.02;

G = tf([wn^2], [1 2*zeta*wn wn^2]);

s = tf('s');

% Controller gains (you can tune these)
Kp = 1;
Kd = 5e-5;
Ki = 200;

% Controllers
C_p   = Kp;
C_pd  = Kp + Kd*s;
C_pid = Kp + Kd*s + Ki/s;

% Open-loop systems
L_p   = C_p   * G;
L_pd  = C_pd  * G;
L_pid = C_pid * G;

% --- Magnitude + Phase comparison ---
figure;
bode(L_p, 'b', L_pd, 'r--', L_pid, 'g-.');
grid on;
legend('P', 'PD', 'PID');
title('Bode Plot Comparison: P vs PD vs PID');

setoptions(gca, 'FreqUnits', 'Hz');

% --- Stability margins ---
figure;
margin(L_p);   title('P Controller');

figure;
margin(L_pd);  title('PD Controller');

figure;
margin(L_pid); title('PID Controller');
