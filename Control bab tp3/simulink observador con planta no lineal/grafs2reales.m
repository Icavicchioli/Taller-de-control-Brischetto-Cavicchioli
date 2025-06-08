%%
% Parámetros de tamaño en pulgadas
ancho = 7.87;  % ~20 cm
alto = 6.3;    % ~16 cm

% Variables de entrada (similares a las tuyas)
angulo = out.angulo;       % Nx2
referencia = out.u;   % Nx1
posicion = out.posicion;     % Nx2
velocidad = out.velocidad;    % Nx2
vel_angular = out.velocidad_angular; % Nx2
muestras = length(out.posicion);
tiempo = 0:0.02:(muestras-1)*0.02;

t1 = 0:Ts:(length(tiempo)-1-1)*0.02;

velocidad_medida = diff(out.posicion(:, 1))./diff(t1);
velocidad_medida = velocidad_medida(:, 1);

% --- Ángulos y referencia ---
fig1 = figure;
set(fig1, 'Units', 'inches', 'Position', [1,1, ancho, alto]);
hold on;
plot(tiempo, angulo(:,1), '-', 'LineWidth', 2);
plot(tiempo, angulo(:,2), '-', 'LineWidth', 2);
plot(tiempo, referencia, 'LineWidth', 2);
title('Ángulo y referencia');
xlabel('Tiempo [s]');
ylabel('Ángulo [rad]');xlim([0 max(tiempo)]);
legend({'Ángulo real', 'Ángulo estimado', 'Referencia'},'Location','northeast');
grid on;
set(fig1, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig1, 'angulo_referencia_real', '-dpdf', '-r300');

% --- Posiciones ---
fig2 = figure;
set(fig2, 'Units', 'inches', 'Position', [1,1, ancho, alto]);
hold on;
plot(tiempo, posicion(:,1), '-', 'LineWidth', 2);
plot(tiempo, posicion(:,2), '-', 'LineWidth', 2);
title('Posición');
xlabel('Tiempo [s]');
ylabel('Posición [m]');xlim([0 max(tiempo)]);
legend({'Posición real', 'Posición estimada'},'Location','northeast');
grid on;
set(fig2, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig2, 'posiciones_real', '-dpdf', '-r300');

% --- Velocidades ---
fig3 = figure;
set(fig3, 'Units', 'inches', 'Position', [1,1, ancho, alto]);
hold on;
plot(t1, velocidad_medida, '-', 'LineWidth', 2);
plot(tiempo, velocidad(:,2), '-', 'LineWidth', 2);
title('Velocidad');
xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');xlim([0 max(tiempo)]);
legend({'Velocidad real', 'Velocidad estimada'},'Location','northeast');
grid on;
set(fig3, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig3, 'velocidades_real', '-dpdf', '-r300');

% --- Velocidad angular ---
fig4 = figure;
set(fig4, 'Units', 'inches', 'Position', [1,1, ancho, alto]);
hold on;
plot(tiempo, vel_angular(:,1), '-', 'LineWidth', 2);
plot(tiempo, vel_angular(:,2), '-', 'LineWidth', 2);
title('Velocidad angular');xlim([0 max(tiempo)]);
xlabel('Tiempo [s]');
ylabel('Velocidad angular [rad/s]');
legend({'Vel. angular real', 'Vel. angular estimada'},'Location','southeast');
grid on;
set(fig4, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig4, 'velocidad_angular_real', '-dpdf', '-r300');
