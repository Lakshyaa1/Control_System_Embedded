clc;
clear;
close all;

% Parameters
fn = 1.5e3;
wn = 2*pi*fn;
zeta = 0.02;

% Transfer function
G = tf([wn^2], [1 2*zeta*wn wn^2]);

% --- Plot with margins ---
figure;
margin(G);
grid on;
title('Open-loop Bode Plot with Stability Margins');

% --- Extract numerical values ---
[GM, PM, Wcg, Wcp] = margin(G);

% Convert to Hz
f_cg = Wcg / (2*pi);
f_cp = Wcp / (2*pi);

% Convert GM to dB
GM_dB = 20*log10(GM);

% Print results
fprintf('Gain Margin: %.2f dB\n', GM_dB);
fprintf('Phase Margin: %.2f degrees\n', PM);
fprintf('Gain Crossover Frequency: %.2f Hz\n', f_cg);
fprintf('Phase Crossover Frequency: %.2f Hz\n', f_cp);
