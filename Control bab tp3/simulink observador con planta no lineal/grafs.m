% veamos las señales obtenidas
angulo = out.ang; %1 es el real, 2 el del observador
referencia = out.ref;
posicion = out.pos;
velocidad = out.vel;
vel_angular = out.velang;
tiempo = linspace(0,250,251)*0.02;

%%
close all;
figure;
fig = figure;
set(fig, 'Units', 'inches', 'Position', [1, 1, 7.87, 6.3]);  % [x y width height]
subplot(2,2,1);
hold on;
plot(tiempo,angulo(:,1),':', 'LineWidth', 2);
plot(tiempo,angulo(:,2),'--', 'LineWidth', 2);
plot(tiempo,referencia, 'LineWidth', 2);
title('Ángulos y referencia');
xlabel('Tiempo [s]');
ylabel('Ángulo [rad]');
legend({'Ángulo real', 'Ángulo estimado','Referencia'});
grid on;
subplot(2,2,2);
hold on;
plot(tiempo,posicion(:,1),':', 'LineWidth', 2);
plot(tiempo,posicion(:,2),'--', 'LineWidth', 2);
title('Posiciones');
xlabel('Tiempo [s]');
ylabel('Posición [m]');
legend({'Posición real', 'Posición estimada'});
grid on;
subplot(2,2,3);
hold on;
plot(tiempo,velocidad(:,1),':', 'LineWidth', 2);
plot(tiempo,velocidad(:,2),'--', 'LineWidth', 2);
title('Velocidades');
xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');
legend({'Velocidad real', 'Velocidad estimada'});
grid on;
subplot(2,2,4);
hold on;
plot(tiempo,vel_angular(:,1),':', 'LineWidth', 2);
plot(tiempo,vel_angular(:,2),'--', 'LineWidth', 2);
title('Velocidad angular');
xlabel('Tiempo [s]');
ylabel('Velocidad angular [rad/s]');
legend({'Vel. angular real', 'Vel. angular estimada'});
grid on;
% Ajustar tamaño al exportar
set(fig, 'PaperUnits', 'inches');
set(fig, 'PaperPosition', [0, 0, 7.87, 6.3]);
set(fig, 'PaperSize', [7.87, 6.3]);
% Guardar como PDF
print(fig, 'mi_figura', '-dpdf', '-r300');
% (opcional) Guardar como imagen PNG de alta resolución
print(fig, 'mi_figura', '-dpng', '-r300');

%%
% Parámetros de tamaño en pulgadas
ancho = 7.87;  % ~20 cm
alto = 6.3;    % ~16 cm

% Variables de entrada (similares a las tuyas)
angulo = out.ang;       % Nx2
referencia = out.ref;   % Nx1
posicion = out.pos;     % Nx2
velocidad = out.vel;    % Nx2
vel_angular = out.velang; % Nx2
tiempo = linspace(0,250,251)*0.02;

% --- Ángulos y referencia ---
fig1 = figure;
set(fig1, 'Units', 'inches', 'Position', [1,1, ancho, alto]);
hold on;
plot(tiempo, angulo(:,1), ':', 'LineWidth', 2);
plot(tiempo, angulo(:,2), '--', 'LineWidth', 2);
plot(tiempo, referencia, 'LineWidth', 2);
title('Ángulo y referencia');
xlabel('Tiempo [s]');
ylabel('Ángulo [rad]');
legend({'Ángulo real', 'Ángulo estimado', 'Referencia'},'Location','northeast');
grid on;
set(fig1, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig1, 'angulo_referencia', '-dpdf', '-r300');

% --- Posiciones ---
fig2 = figure;
set(fig2, 'Units', 'inches', 'Position', [1,1, ancho, alto]);
hold on;
plot(tiempo, posicion(:,1), ':', 'LineWidth', 2);
plot(tiempo, posicion(:,2), '--', 'LineWidth', 2);
title('Posición');
xlabel('Tiempo [s]');
ylabel('Posición [m]');
legend({'Posición real', 'Posición estimada'},'Location','northeast');
grid on;
set(fig2, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig2, 'posiciones', '-dpdf', '-r300');

% --- Velocidades ---
fig3 = figure;
set(fig3, 'Units', 'inches', 'Position', [1,1, ancho, alto]);
hold on;
plot(tiempo, velocidad(:,1), ':', 'LineWidth', 2);
plot(tiempo, velocidad(:,2), '--', 'LineWidth', 2);
title('Velocidad');
xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');
legend({'Velocidad real', 'Velocidad estimada'},'Location','northeast');
grid on;
set(fig3, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig3, 'velocidades', '-dpdf', '-r300');

% --- Velocidad angular ---
fig4 = figure;
set(fig4, 'Units', 'inches', 'Position', [1,1, ancho, alto]);
hold on;
plot(tiempo, vel_angular(:,1), ':', 'LineWidth', 2);
plot(tiempo, vel_angular(:,2), '--', 'LineWidth', 2);
title('Velocidad angular');
xlabel('Tiempo [s]');
ylabel('Velocidad angular [rad/s]');
legend({'Vel. angular real', 'Vel. angular estimada'},'Location','southeast');
grid on;
set(fig4, 'PaperUnits', 'inches', 'PaperPosition', [0 0 ancho alto], 'PaperSize', [ancho alto]);
print(fig4, 'velocidad_angular', '-dpdf', '-r300');
