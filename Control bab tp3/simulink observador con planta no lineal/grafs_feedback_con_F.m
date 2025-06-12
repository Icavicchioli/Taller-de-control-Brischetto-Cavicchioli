%%
% Parámetros de tamaño en pulgadas
ancho = 7.87;  % ~20 cm
alto = 6.3;    % ~16 cm
limit_tiempo = 12.5;

% Variables
tiempo1 = tiempo_ard;

% --- Figura 1: Señales Arduino (reales) ---
fig_real = figure;
set(fig_real, 'Units', 'inches', 'Position', [1,1, ancho, alto]);

subplot(2,2,1);
plot(tiempo1, angulo_arduino(1:655), 'b', 'LineWidth', 1);
title('Ángulo - Observador Arduino');
xlabel('Tiempo [s]');
ylabel('[rad]');
xlim([0 limit_tiempo]);
grid on;

subplot(2,2,2);
plot(tiempo1, posicion_arduino(1:655), 'b', 'LineWidth', 1);
title('Posición - Observador Arduino');
xlabel('Tiempo [s]');
ylabel('[m]');
xlim([0 limit_tiempo]);
grid on;

subplot(2,2,3);
plot(tiempo1, velocidad_arduino(1:655), 'b', 'LineWidth', 1);
title('Velocidad - Observador Arduino');
xlabel('Tiempo [s]');
ylabel('[m/s]');
xlim([0 limit_tiempo]);
grid on;

subplot(2,2,4);
plot(tiempo1, velocidadAng_arduino(1:655), 'b', 'LineWidth', 1);
title('Velocidad angular - Observador Arduino');
xlabel('Tiempo [s]');
ylabel('[rad/s]');
xlim([0 limit_tiempo]);
grid on;

set(fig_real, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig_real, 'senales_arduino', '-dpdf', '-r300');

% --- Figura 2: Señales estimadas ---
fig_estimado = figure;
set(fig_estimado, 'Units', 'inches', 'Position', [1,1, ancho, alto]);

subplot(2,2,1);
plot(tiempo1, angulo(1:655), 'r', 'LineWidth', 1);
title('Ángulo - Observador Simulink');
xlabel('Tiempo [s]');
ylabel('[rad]');
xlim([0 limit_tiempo]);
grid on;

subplot(2,2,2);
plot(tiempo1, posicion(1:655), 'r', 'LineWidth', 1);
title('Posición - Observador Simulink');
xlabel('Tiempo [s]');
ylabel('[m]');
xlim([0 limit_tiempo]);
grid on;

subplot(2,2,3);
plot(tiempo1, velocidad(1:655), 'r', 'LineWidth', 1);
title('Velocidad - Observador Simulink');
xlabel('Tiempo [s]');
ylabel('[m/s]');
xlim([0 limit_tiempo]);
grid on;

subplot(2,2,4);
plot(tiempo1, velocidad_ang(1:655), 'r', 'LineWidth', 1);
title('Velocidad angular - Observador Simulink');
xlabel('Tiempo [s]');
ylabel('[rad/s]');
xlim([0 limit_tiempo]);
grid on;

set(fig_estimado, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig_estimado, 'senales_estimadas', '-dpdf', '-r300');
