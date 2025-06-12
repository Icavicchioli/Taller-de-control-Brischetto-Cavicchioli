%%
% Parámetros de tamaño en pulgadas
ancho = 7.87;  % ~20 cm
alto = 6.3;    % ~16 cm
limit_tiempo = 19;

% Variables
tiempo1 = tiempo_arduino;

% --- Figura 1: Señales Arduino (reales) ---
fig_real = figure;
set(fig_real, 'Units', 'inches', 'Position', [1,1, ancho, alto]);

subplot(2,2,1);
plot(tiempo1, angulo_arduino, 'b', 'LineWidth', 1);
title('Ángulo - Observador Arduino');
xlabel('Tiempo [s]');
ylabel('[rad]');
xlim([0 limit_tiempo]);
grid on;

subplot(2,2,2);
hold on;
plot(tiempo1, posicion_arduino, 'b', 'LineWidth', 1);
plot(tiempo1,ref_arduino , 'k', 'LineWidth', 1);
hold off;
title('Posición - Observador Arduino');
xlabel('Tiempo [s]');
ylabel('[m]');
xlim([0 limit_tiempo]);
grid on;

subplot(2,2,3);
plot(tiempo1,vel_arduino,'b','LineWidth',1);
title('Velocidad - Observador Arduino');
xlabel('Tiempo [s]');
ylabel('[m/s]');
xlim([0 limit_tiempo]);
grid on;

subplot(2,2,4);
plot(tiempo1, velang_arduino, 'b', 'LineWidth', 1);
title('Velocidad angular - Observador Arduino');
xlabel('Tiempo [s]');
ylabel('[rad/s]');
xlim([0 limit_tiempo]);
grid on;

set(fig_real, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig_real, 'senales_arduino_con_feed', '-dpdf', '-r300');

%% --- Figura 2: Señales estimadas ---
fig_estimado = figure;
set(fig_estimado, 'Units', 'inches', 'Position', [1,1, ancho, alto]);

subplot(2,2,1);
plot(tiempo_simulink, angulo_obs, 'r', 'LineWidth', 1);
title('Ángulo - Observador Simulink');
xlabel('Tiempo [s]');
ylabel('[rad]');
xlim([0 limit_tiempo]);
grid on;

subplot(2,2,2);
hold on;
plot(tiempo_simulink, dist_obs, 'r', 'LineWidth', 1);
plot(tiempo_simulink,ref_pos_obs , 'k', 'LineWidth', 1);
hold off;
title('Posición - Observador Simulink');
xlabel('Tiempo [s]');
ylabel('[m]');
xlim([0 limit_tiempo]);
grid on;

subplot(2,2,3);
plot(tiempo_simulink, vel_obs, 'r', 'LineWidth', 1);
title('Velocidad - Observador Simulink');
xlabel('Tiempo [s]');
ylabel('[m/s]');
xlim([0 limit_tiempo]);
grid on;

subplot(2,2,4);
plot(tiempo_simulink, velang_obs, 'r', 'LineWidth', 1);
title('Velocidad angular - Observador Simulink');
xlabel('Tiempo [s]');
ylabel('[rad/s]');
xlim([0 limit_tiempo]);
grid on;

set(fig_estimado, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig_estimado, 'senales_estimadas_con_feed', '-dpdf', '-r300');
