%%
% ParÃ¡metros de tamaÃ±o en pulgadas
ancho = 7.87;  % ~20 cm
alto = 6.3;    % ~16 cm
limit_tiempo = 12.5;
% Variables de entrada (similares a las tuyas)
angulo1 = [angulo_arduino(1:655) angulo(1:655)];       % Nx2
%referencia = out.ref(:,1);   % Nx1
%referencia = zeros(1001);
posicion1 = [posicion_arduino(1:655) posicion(1:655)];     % Nx2
velocidad1 = [ velocidad_arduino(1:655) velocidad(1:655)];    % Nx2
vel_angular1 = [velocidadAng_arduino(1:655) velocidad_ang(1:655)]; % Nx2
tiempo1=tiempo_ard;
%%

% --- Ã?ngulos y referencia ---
fig1 = figure;
set(fig1, 'Units', 'inches', 'Position', [1,1, ancho, alto]);
hold on;
plot(tiempo1, angulo1(:,1), '-', 'LineWidth', 2);
plot(tiempo1, angulo1(:,2), '-', 'LineWidth', 2);
title('Ángulo');
xlabel('Tiempo [s]');
ylabel('Ángulo [rad]');
xlim([0 limit_tiempo]);
legend({'Ángulo real', 'Ángulo estimado'},'Location','northeast');
grid on;
set(fig1, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig1, 'angulo_referencia_real', '-dpdf', '-r300');

% --- Posiciones ---
fig2 = figure;
set(fig2, 'Units', 'inches', 'Position', [1,1, ancho, alto]);
hold on;
plot(tiempo1, posicion1(:,1), '-', 'LineWidth', 2);
plot(tiempo1, posicion1(:,2), '-', 'LineWidth', 2);
title('Posición - Referencia cero');
xlabel('Tiempo [s]');
ylabel('Posición [m]');
xlim([0 limit_tiempo]);
legend({'Posición real', 'Posición estimada'},'Location','northeast');
grid on;
set(fig2, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig2, 'posiciones_real', '-dpdf', '-r300');

% --- Velocidades ---
fig3 = figure;
set(fig3, 'Units', 'inches', 'Position', [1,1, ancho, alto]);
hold on;
plot(tiempo1, velocidad1(:,1), '-', 'LineWidth', 2);
plot(tiempo1, velocidad1(:,2), '-', 'LineWidth', 2);
title('Velocidad');
xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');
xlim([0 limit_tiempo]);
legend({'Velocidad real', 'Velocidad estimada'},'Location','northeast');
grid on;
set(fig3, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig3, 'velocidades_real', '-dpdf', '-r300');

% --- Velocidad angular ---
fig4 = figure;
set(fig4, 'Units', 'inches', 'Position', [1,1, ancho, alto]);
hold on;
plot(tiempo1, vel_angular1(:,1), '-', 'LineWidth', 2);
plot(tiempo1, vel_angular1(:,2), '-', 'LineWidth', 2);
title('Velocidad angular');
xlim([0 limit_tiempo]);
xlabel('Tiempo [s]');
ylabel('Velocidad angular [rad/s]');
legend({'Vel. angular real', 'Vel. angular estimada'},'Location','southeast');
grid on;
set(fig4, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig4, 'velocidad_angular_real', '-dpdf', '-r300');
